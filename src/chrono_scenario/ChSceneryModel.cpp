// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
//
// Static 3D scenery loaded from an instanced placement manifest.
//
// =============================================================================

#include <algorithm>
#include <fstream>
#include <iostream>
#include <ostream>
#include <sstream>

#include "chrono/assets/ChVisualShapeModelFile.h"
#include "chrono/core/ChTypes.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

#include "chrono_scenario/ChSceneryModel.h"

namespace chrono {
namespace scenario {

namespace {

/// Directory containing the manifest; mesh paths are relative to it.
std::string DirectoryOf(const std::string& file) {
    size_t slash = file.find_last_of("/\\");
    return slash == std::string::npos ? std::string(".") : file.substr(0, slash);
}

bool FileExists(const std::string& path) {
    std::ifstream f(path);
    return f.good();
}

/// Count triangles in a Wavefront OBJ without building a mesh.
///
/// Chrono defers actual mesh loading to the visual system, so this is the only cheap way to know
/// an asset's cost before deciding whether to keep it. Reading a line at a time avoids pulling a
/// large file into memory just to reject it.
unsigned int CountObjTriangles(const std::string& path) {
    std::ifstream f(path);
    if (!f.good())
        return 0;

    unsigned int tris = 0;
    std::string line;
    while (std::getline(f, line)) {
        if (line.size() < 2 || line[0] != 'f' || line[1] != ' ')
            continue;
        // A face with n vertices fan-triangulates to n-2 triangles.
        unsigned int verts = 0;
        bool in_tok = false;
        for (size_t i = 2; i < line.size(); i++) {
            bool sp = (line[i] == ' ' || line[i] == '\t' || line[i] == '\r');
            if (!sp && !in_tok) {
                verts++;
                in_tok = true;
            } else if (sp) {
                in_tok = false;
            }
        }
        if (verts >= 3)
            tris += verts - 2;
    }
    return tris;
}

bool Listed(const std::vector<std::string>& list, const std::string& value) {
    return std::find(list.begin(), list.end(), value) != list.end();
}

/// Instances differing only in placement share a shape; differing scales cannot, because scale
/// lives on the shape rather than the instance frame. Quantising keeps near-identical scales
/// together instead of producing a shape per instance from floating-point noise.
std::string ScaleKey(const ChVector3d& s) {
    std::ostringstream os;
    os << std::llround(s.x() * 1000) << '_' << std::llround(s.y() * 1000) << '_'
       << std::llround(s.z() * 1000);
    return os.str();
}

}  // namespace

ChSceneryModel::ChSceneryModel()
    : m_num_assets(0), m_num_instances(0), m_num_skipped(0), m_num_missing(0) {}

bool ChSceneryModel::Load(ChSystem& sys, const std::string& manifest_file, const Options& options) {
    std::ifstream in(manifest_file);
    if (!in.good()) {
        std::cerr << "ChSceneryModel: cannot open manifest " << manifest_file << std::endl;
        return false;
    }

    rapidjson::IStreamWrapper isw(in);
    rapidjson::Document doc;
    doc.ParseStream(isw);
    if (doc.HasParseError() || !doc.HasMember("assets") || !doc.HasMember("instances")) {
        std::cerr << "ChSceneryModel: " << manifest_file << " is not a valid scenery manifest"
                  << std::endl;
        return false;
    }

    const std::string dir = DirectoryOf(manifest_file);

    // Resolve every asset once: its mesh path, its colour, and whether it is usable at all.
    struct Asset {
        std::string name;
        std::string mesh;
        ChColor colour{0.62f, 0.62f, 0.64f};
        bool has_colour = false;
        bool usable = false;
    };

    std::vector<Asset> assets;
    assets.reserve(doc["assets"].Size());
    for (const auto& a : doc["assets"].GetArray()) {
        Asset asset;
        if (a.HasMember("name"))
            asset.name = a["name"].GetString();
        if (a.HasMember("mesh"))
            asset.mesh = dir + "/" + a["mesh"].GetString();

        if (a.HasMember("colour") && a["colour"].IsArray() && a["colour"].Size() >= 3) {
            asset.colour = ChColor(a["colour"][0].GetFloat(), a["colour"][1].GetFloat(),
                                   a["colour"][2].GetFloat());
            asset.has_colour = true;
        }

        if (asset.mesh.empty() || !FileExists(asset.mesh)) {
            m_num_missing++;
        } else if (options.max_triangles_per_asset > 0 &&
                   CountObjTriangles(asset.mesh) > options.max_triangles_per_asset) {
            // Deliberately dropped rather than missing; counted with the skips below.
        } else {
            asset.usable = true;
            m_num_assets++;
        }

        assets.push_back(asset);
    }

    // One body per group keeps categories independently controllable; a single body is marginally
    // cheaper. Either way these never enter the solver.
    auto make_body = [&sys](const std::string& name) {
        auto body = chrono_types::make_shared<ChBody>();
        body->SetName(("scenery_" + name).c_str());
        body->SetFixed(true);
        body->EnableCollision(false);
        sys.Add(body);
        return body;
    };

    std::map<std::string, std::shared_ptr<ChBody>> group_bodies;
    std::shared_ptr<ChBody> single_body;
    if (!options.body_per_group)
        single_body = make_body("all");

    // A shape is shared by every instance of the same asset at the same scale, so the geometry is
    // stored once however many times it appears.
    std::map<std::string, std::shared_ptr<ChVisualShapeModelFile>> shapes;

    for (const auto& inst : doc["instances"].GetArray()) {
        if (!inst.HasMember("asset"))
            continue;

        unsigned int ai = inst["asset"].GetUint();
        std::string group = inst.HasMember("group") ? inst["group"].GetString() : "default";

        if (ai >= assets.size() || !assets[ai].usable) {
            m_num_skipped++;
            continue;
        }
        if (!options.include_groups.empty() && !Listed(options.include_groups, group)) {
            m_num_skipped++;
            continue;
        }
        if (Listed(options.exclude_groups, group)) {
            m_num_skipped++;
            continue;
        }

        ChVector3d pos(0, 0, 0);
        if (inst.HasMember("pos") && inst["pos"].Size() >= 3)
            pos = ChVector3d(inst["pos"][0].GetDouble(), inst["pos"][1].GetDouble(),
                             inst["pos"][2].GetDouble());

        ChQuaterniond rot(1, 0, 0, 0);
        if (inst.HasMember("rot") && inst["rot"].Size() >= 4)
            rot = ChQuaterniond(inst["rot"][0].GetDouble(), inst["rot"][1].GetDouble(),
                                inst["rot"][2].GetDouble(), inst["rot"][3].GetDouble());
        rot.Normalize();

        ChVector3d scale(1, 1, 1);
        if (inst.HasMember("scale") && inst["scale"].Size() >= 3)
            scale = ChVector3d(inst["scale"][0].GetDouble(), inst["scale"][1].GetDouble(),
                               inst["scale"][2].GetDouble());

        std::string key = assets[ai].name + "#" + ScaleKey(scale);
        auto it = shapes.find(key);
        if (it == shapes.end()) {
            auto shape = chrono_types::make_shared<ChVisualShapeModelFile>();
            shape->SetFilename(assets[ai].mesh);
            shape->SetScale(scale);
            if (assets[ai].has_colour)
                shape->SetColor(assets[ai].colour);
            it = shapes.emplace(key, shape).first;
        }

        std::shared_ptr<ChBody> body;
        if (options.body_per_group) {
            auto bit = group_bodies.find(group);
            if (bit == group_bodies.end())
                bit = group_bodies.emplace(group, make_body(group)).first;
            body = bit->second;
        } else {
            body = single_body;
        }

        body->AddVisualShape(it->second, ChFrame<double>(pos, rot));
        m_num_instances++;
        m_per_group[group]++;
    }

    m_bodies.clear();
    if (single_body)
        m_bodies.push_back(single_body);
    for (const auto& kv : group_bodies)
        m_bodies.push_back(kv.second);

    return true;
}

void ChSceneryModel::ReportTo(std::ostream& out) const {
    out << "  scenery: " << m_num_instances << " instances from " << m_num_assets << " meshes on "
        << m_bodies.size() << (m_bodies.size() == 1 ? " body" : " bodies") << "\n";
    for (const auto& kv : m_per_group)
        out << "    " << kv.second << "  " << kv.first << "\n";
    if (m_num_missing)
        out << "    (" << m_num_missing << " assets missing from disk)\n";
    if (m_num_skipped)
        out << "    (" << m_num_skipped << " instances filtered or unusable)\n";
}

}  // end namespace scenario
}  // end namespace chrono

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

#include "chrono/assets/ChVisualMaterial.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"
#include "chrono/core/ChTypes.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"

#include "chrono_thirdparty/rapidjson/document.h"
#include "chrono_thirdparty/rapidjson/istreamwrapper.h"

#include "chrono_vehicle/utils/ChSceneryModel.h"

namespace chrono {
namespace vehicle {

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
    : m_num_assets(0), m_num_instances(0), m_num_skipped(0), m_num_missing(0), m_num_textures(0) {}

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

    // Resolve every asset once. An asset is a list of parts, each a single-material mesh; see the
    // header for why the split happens upstream rather than here.
    struct Part {
        std::string mesh;     ///< absolute path
        std::string texture;    ///< base colour, absolute path; empty when there is none
        std::string normal;     ///< normal map, absolute path
        std::string roughness;  ///< roughness map, absolute path
        std::string metallic;   ///< metallic map, absolute path
        ChColor colour{0.62f, 0.62f, 0.64f};
        ChColor specular{0.05f, 0.05f, 0.05f};
        float shininess = 10.0f;
    };
    struct Asset {
        std::string name;
        std::vector<Part> parts;
        bool usable = false;
    };

    std::vector<Asset> assets;
    assets.reserve(doc["assets"].Size());
    for (const auto& a : doc["assets"].GetArray()) {
        Asset asset;
        if (a.HasMember("name"))
            asset.name = a["name"].GetString();

        if (a.HasMember("parts") && a["parts"].IsArray()) {
            for (const auto& p : a["parts"].GetArray()) {
                Part part;
                if (p.HasMember("mesh"))
                    part.mesh = dir + "/" + p["mesh"].GetString();
                if (p.HasMember("texture") && p["texture"].IsString())
                    part.texture = dir + "/" + p["texture"].GetString();
                if (p.HasMember("normal") && p["normal"].IsString())
                    part.normal = dir + "/" + p["normal"].GetString();
                if (p.HasMember("roughness") && p["roughness"].IsString())
                    part.roughness = dir + "/" + p["roughness"].GetString();
                if (p.HasMember("metallic") && p["metallic"].IsString())
                    part.metallic = dir + "/" + p["metallic"].GetString();
                if (p.HasMember("colour") && p["colour"].IsArray() && p["colour"].Size() >= 3)
                    part.colour = ChColor(p["colour"][0].GetFloat(), p["colour"][1].GetFloat(),
                                          p["colour"][2].GetFloat());
                if (p.HasMember("ks") && p["ks"].IsArray() && p["ks"].Size() >= 3)
                    part.specular = ChColor(p["ks"][0].GetFloat(), p["ks"][1].GetFloat(),
                                            p["ks"][2].GetFloat());
                if (p.HasMember("ns") && p["ns"].IsNumber())
                    part.shininess = p["ns"].GetFloat();

                if (part.mesh.empty() || !FileExists(part.mesh))
                    continue;
                if (options.max_triangles_per_asset > 0 &&
                    CountObjTriangles(part.mesh) > options.max_triangles_per_asset)
                    continue;
                asset.parts.push_back(part);
            }
        }

        if (asset.parts.empty()) {
            m_num_missing++;
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
    // stored once however many times it appears. Meshes are cached separately because several
    // scales of one asset can share the loaded triangles.
    std::map<std::string, std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>>> shapes;
    std::map<std::string, std::shared_ptr<ChTriangleMeshConnected>> meshes;
    std::map<std::string, std::shared_ptr<ChVisualMaterial>> part_materials;

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
            // Deliberately triangle meshes rather than a ChVisualShapeModelFile. A model file is
            // handed straight to the visual system's own importer, which in Chrono's VSG backend
            // does not apply the MTL -- Chrono itself never parses MTL at all, so a textured OBJ
            // renders untextured and, because a textured surface carries a white base colour,
            // washed out. Loading the triangles here and attaching a ChVisualMaterial is the route
            // that actually carries a texture, and it is equally visible to the sensor pipeline.
            std::vector<std::shared_ptr<ChVisualShapeTriangleMesh>> parts;
            for (const auto& p : assets[ai].parts) {
                auto mit = meshes.find(p.mesh);
                if (mit == meshes.end()) {
                    // UVs are not loaded by default and are exactly what a texture needs.
                    auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(p.mesh, true, true);
                    mit = meshes.emplace(p.mesh, mesh).first;
                }
                if (!mit->second || mit->second->GetNumTriangles() == 0)
                    continue;

                // One material object per part, reused by every scale of the asset. Building a
                // fresh material for each scale would be harmless on its own, but the visual
                // system keys its geometry cache on (mesh, materials), so distinct material
                // objects defeat sharing between placements. Foliage makes that decisive: the
                // authoring tool randomises scale per tree, so a per-scale material means a
                // per-tree vertex buffer.
                auto matit = part_materials.find(p.mesh);
                if (matit == part_materials.end()) {
                    auto mat = chrono_types::make_shared<ChVisualMaterial>();
                    mat->SetDiffuseColor(p.colour);
                    mat->SetSpecularColor(p.specular);
                    mat->SetSpecularExponent(p.shininess);
                    // Wavefront Ns runs 0..1000 and is the inverse sense of PBR roughness.
                    // Mapping it that way keeps asphalt matte and glass sharp instead of
                    // flattening both.
                    mat->SetRoughness(1.0f - std::min(1.0f, p.shininess / 100.0f));
                    if (!p.texture.empty() && FileExists(p.texture)) {
                        mat->SetKdTexture(p.texture);
                        m_num_textures++;
                    }
                    // Relief is what stops a fine-grained surface averaging out to flat colour
                    // at any distance where its grain falls below a pixel.
                    if (!p.normal.empty() && FileExists(p.normal))
                        mat->SetNormalMapTexture(p.normal);
                    // Chrono's VSG backend packs these into one texture and needs both or
                    // neither.
                    if (!p.roughness.empty() && FileExists(p.roughness) && !p.metallic.empty() &&
                        FileExists(p.metallic)) {
                        mat->SetRoughnessTexture(p.roughness);
                        mat->SetMetallicTexture(p.metallic);
                    }
                    matit = part_materials.emplace(p.mesh, mat).first;
                }

                auto shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
                // load_materials = false is load-bearing. The default re-runs tinyobj over the
                // OBJ on every call, hunting for an MTL -- so building N shapes from one cached
                // mesh re-parses that file N times. With 4313 shapes over 575 files that was
                // 131 s of the 133 s spent here. The materials are attached explicitly below, so
                // the MTL pass has nothing to contribute anyway.
                shape->SetMesh(mit->second, false);
                shape->SetName(assets[ai].name);
                shape->SetScale(scale);
                shape->SetMutable(false);
                shape->AddMaterial(matit->second);
                parts.push_back(shape);
            }
            if (parts.empty()) {
                m_num_skipped++;
                continue;
            }
            it = shapes.emplace(key, parts).first;
        }

        std::shared_ptr<ChBody> body;
        if (options.body_per_group) {
            std::string bkey = group;
            auto bit = group_bodies.find(bkey);
            if (bit == group_bodies.end())
                bit = group_bodies.emplace(bkey, make_body(bkey)).first;
            body = bit->second;
        } else {
            body = single_body;
        }

        ChFrame<double> frame(pos + options.position_offset, rot);
        for (const auto& part : it->second)
            body->AddVisualShape(part, frame);

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
    out << "  scenery: " << m_num_instances << " instances from " << m_num_assets << " bodies="
        << m_bodies.size() << ", " << m_num_textures << " textured surfaces\n";
    for (const auto& kv : m_per_group)
        out << "    " << kv.second << "  " << kv.first << "\n";
    if (m_num_missing)
        out << "    (" << m_num_missing << " assets missing from disk)\n";
    if (m_num_skipped)
        out << "    (" << m_num_skipped << " instances filtered or unusable)\n";
}

}  // end namespace vehicle
}  // end namespace chrono

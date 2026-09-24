// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Demo for working with Eigen references.
// (Mostly for Chrono developers)
//
// =============================================================================

#include <algorithm>
#include <iostream>
#include <random>
#include <set>
#include <vector>

#include "chrono/core/ChMatrix.h"
#include "chrono/core/ChSparsityPatternLearner.h"

#include "gtest/gtest.h"

const double precision = 1e-8;

using std::cout;
using std::endl;
using namespace chrono;

// ------------------------------------------------------------------

TEST(SparseMatrix, pattern_learner) {
    ChSparsityPatternLearner spl(3, 3);

    spl.SetElement(0, 0, 1.1);
    spl.SetElement(1, 1, 2.2);
    spl.SetElement(1, 0, 2.1);
    spl.SetElement(2, 2, 3.3);

    ChSparseMatrix spmat_learned;
    spl.Apply(spmat_learned);

    ASSERT_TRUE(spmat_learned.outerIndexPtr()[0] == 0);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[1] == 1);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[2] == 3);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[3] == 4);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[0] == 0);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[1] == 0);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[2] == 1);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[3] == 2);

    ChSparseMatrix spmat_mirror(3, 3);

    spmat_mirror.SetElement(0, 0, 1.1);
    spmat_mirror.SetElement(1, 1, 2.2);
    spmat_mirror.SetElement(1, 0, 2.1);
    spmat_mirror.SetElement(2, 2, 3.3);
    spmat_mirror.makeCompressed();

    ASSERT_TRUE(spmat_learned.outerIndexPtr()[0] == spmat_mirror.outerIndexPtr()[0]);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[1] == spmat_mirror.outerIndexPtr()[1]);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[2] == spmat_mirror.outerIndexPtr()[2]);
    ASSERT_TRUE(spmat_learned.outerIndexPtr()[3] == spmat_mirror.outerIndexPtr()[3]);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[0] == spmat_mirror.innerIndexPtr()[0]);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[1] == spmat_mirror.innerIndexPtr()[1]);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[2] == spmat_mirror.innerIndexPtr()[2]);
    ASSERT_TRUE(spmat_learned.innerIndexPtr()[3] == spmat_mirror.innerIndexPtr()[3]);

    ASSERT_NEAR(spmat_mirror.valuePtr()[0], 1.1, precision);
    ASSERT_NEAR(spmat_mirror.valuePtr()[1], 2.1, precision);
    ASSERT_NEAR(spmat_mirror.valuePtr()[2], 2.2, precision);
    ASSERT_NEAR(spmat_mirror.valuePtr()[3], 3.3, precision);
}

// Pattern learned from many unordered and duplicated insertions must match a brute-force reference, both in the
// per-row counts reserved by Apply and in the final compressed pattern of the matrix loaded afterwards.
TEST(SparseMatrix, pattern_learner_duplicates) {
    const int n = 200;
    const int num_insertions = 20000;

    std::mt19937 rng(42);
    std::uniform_int_distribution<int> idx(0, n - 1);
    std::vector<std::pair<int, int>> entries;
    for (int k = 0; k < num_insertions; k++)
        entries.push_back({idx(rng), idx(rng)});
    // Make every entry appear at least twice, in a different order
    auto shuffled = entries;
    std::shuffle(shuffled.begin(), shuffled.end(), rng);
    entries.insert(entries.end(), shuffled.begin(), shuffled.end());

    std::vector<std::set<int>> reference(n);
    for (const auto& e : entries)
        reference[e.first].insert(e.second);

    ChSparsityPatternLearner spl(n, n);
    for (const auto& e : entries)
        spl.SetElement(e.first, e.second, 1.0);

    ChSparseMatrix spmat_learned;
    spl.Apply(spmat_learned);

    // Apply reserves exactly the unique column indices of each row, in increasing order
    int offset = 0;
    for (int i = 0; i < n; i++) {
        ASSERT_EQ(spmat_learned.outerIndexPtr()[i], offset);
        for (int col : reference[i]) {
            ASSERT_EQ(spmat_learned.innerIndexPtr()[offset], col);
            offset++;
        }
    }
    ASSERT_EQ(spmat_learned.outerIndexPtr()[n], offset);

    // Loading the values afterwards yields the same pattern as a matrix built without the learner
    ChSparseMatrix spmat_mirror(n, n);
    for (const auto& e : entries) {
        spmat_learned.SetElement(e.first, e.second, e.first + 0.001 * e.second);
        spmat_mirror.SetElement(e.first, e.second, e.first + 0.001 * e.second);
    }
    spmat_learned.makeCompressed();
    spmat_mirror.makeCompressed();

    ASSERT_EQ(spmat_learned.nonZeros(), spmat_mirror.nonZeros());
    for (int i = 0; i <= n; i++)
        ASSERT_EQ(spmat_learned.outerIndexPtr()[i], spmat_mirror.outerIndexPtr()[i]);
    for (int k = 0; k < spmat_mirror.nonZeros(); k++) {
        ASSERT_EQ(spmat_learned.innerIndexPtr()[k], spmat_mirror.innerIndexPtr()[k]);
        ASSERT_EQ(spmat_learned.valuePtr()[k], spmat_mirror.valuePtr()[k]);
    }
}

/**
 * @file control_tests.cpp
 * @brief Unit tests for Control and ControlSeq structures.
 */

#include <arcgen/core/control.hpp>
#include <gtest/gtest.h>

using namespace arcgen::core;

/// @brief Verify DrivingDirection logic.
TEST (ControlTests, Direction)
{
    Control fwd{0.0, 10.0};
    EXPECT_EQ (fwd.direction (), DrivingDirection::Forward);

    Control rev{0.0, -10.0};
    EXPECT_EQ (rev.direction (), DrivingDirection::Reverse);

    Control neu{0.0, 0.0};
    EXPECT_EQ (neu.direction (), DrivingDirection::Neutral);
}

/// @brief Verify Control equality operator.
TEST (ControlTests, Equality)
{
    Control c1{0.1, 5.0};
    Control c2{0.1, 5.0};
    Control c3{0.2, 5.0};
    EXPECT_EQ (c1, c2);
    EXPECT_NE (c1, c3);
}

/// @brief Verify basic ControlSeq operations (push_back, length accumulation).
TEST (ControlTests, SequenceBasic)
{
    ControlSeq<3> seq;
    EXPECT_EQ (seq.n, 0);
    EXPECT_EQ (seq.length, 0.0);

    EXPECT_TRUE (seq.push_back ({0.0, 10.0}));
    EXPECT_EQ (seq.n, 1);
    EXPECT_EQ (seq.length, 10.0);

    EXPECT_TRUE (seq.push_back ({0.1, -5.0}));
    EXPECT_EQ (seq.n, 2);
    EXPECT_EQ (seq.length, 15.0); // 10 + |-5|

    EXPECT_TRUE (seq.push_back ({0.0, 1.0}));
    EXPECT_EQ (seq.n, 3);
    EXPECT_EQ (seq.length, 16.0);

    // Overflow
#ifndef NDEBUG
    EXPECT_DEATH (seq.push_back ({0.0, 1.0}), "ControlSeq overflow");
#else
    EXPECT_FALSE (seq.push_back ({0.0, 1.0}));
#endif
    EXPECT_EQ (seq.n, 3);
    EXPECT_EQ (seq.length, 16.0);
}

/// @brief Verify clearing a sequence resets count and length.
TEST (ControlTests, SequenceClear)
{
    ControlSeq<5> seq;
    seq.push_back ({1.0, 1.0});
    seq.clear ();
    EXPECT_EQ (seq.n, 0);
    EXPECT_EQ (seq.length, 0.0);
}

/// @brief Verify ControlSeq equality.
TEST (ControlTests, SequenceEquality)
{
    ControlSeq<3> s1;
    ControlSeq<3> s2;
    s1.push_back ({0.1, 10.0});
    s2.push_back ({0.1, 10.0});
    EXPECT_EQ (s1, s2);

    s1.push_back ({0.0, 5.0});
    EXPECT_NE (s1, s2);

    s2.push_back ({0.0, 5.0});
    EXPECT_EQ (s1, s2);
}

/// @brief Verify behavior of zero-capacity sequence.
TEST (ControlTests, SequenceZeroCapacity)
{
    ControlSeq<0> seq;
    EXPECT_EQ (seq.n, 0);
    EXPECT_EQ (seq.length, 0.0);
    // Should fail to push immediately
#ifndef NDEBUG
    EXPECT_DEATH (seq.push_back ({0, 0}), "ControlSeq overflow");
#else
    EXPECT_FALSE (seq.push_back ({0, 0}));
#endif
}

/// @brief Verify filling sequence exactly to capacity.
TEST (ControlTests, SequenceFillExact)
{
    ControlSeq<2> seq;
    EXPECT_TRUE (seq.push_back ({0.1, 1.0}));
    EXPECT_TRUE (seq.push_back ({0.2, 1.0}));
    EXPECT_EQ (seq.n, 2);
    // Next one fails
#ifndef NDEBUG
    EXPECT_DEATH (seq.push_back ({0.3, 1.0}), "ControlSeq overflow");
#else
    EXPECT_FALSE (seq.push_back ({0.3, 1.0}));
#endif
}

/// @brief Verify copy construction.
TEST (ControlTests, SequenceCopy)
{
    ControlSeq<3> s1;
    s1.push_back ({0.1, 1.0});
    ControlSeq<3> s2 = s1;
    EXPECT_EQ (s1, s2);
    EXPECT_EQ (s2.n, 1);
    EXPECT_EQ (s2.length, 1.0);
}

/// @brief Verify direction logic for edge cases (min/max/epsilon).
TEST (ControlTests, DirectionEdges)
{
    Control c{0.0, std::numeric_limits<double>::min ()};
    EXPECT_EQ (c.direction (), DrivingDirection::Forward);

    c.arcLength = -std::numeric_limits<double>::min ();
    EXPECT_EQ (c.direction (), DrivingDirection::Reverse);
}

/// @brief Verify const view/span access to sequence.
TEST (ControlTests, SequenceView)
{
    ControlSeq<5> seq;
    seq.push_back ({1.0, 2.0});
    seq.push_back ({3.0, 4.0});

    auto v = seq.view ();
    EXPECT_EQ (v.size (), 2);
    EXPECT_EQ (v[0].curvature, 1.0);
    EXPECT_EQ (v[1].curvature, 3.0);

    seq.clear ();
    EXPECT_TRUE (seq.view ().empty ());
}

TEST (ControlTests, SequenceMaxCapacity)
{
    ControlSeq<255> seq;
    for (int i = 0; i < 255; ++i)
    {
        EXPECT_TRUE (seq.push_back ({0.0, 1.0}));
    }
    EXPECT_EQ (seq.n, 255);
    EXPECT_EQ (seq.length, 255.0);

    // Overflow at max
#ifndef NDEBUG
    EXPECT_DEATH (seq.push_back ({0.0, 1.0}), "ControlSeq overflow");
#else
    EXPECT_FALSE (seq.push_back ({0.0, 1.0}));
#endif
}

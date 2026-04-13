#pragma once

#include <gtest/gtest.h>
#include <cloud.h>

// 断言 Cloud 非空
#define ASSERT_CLOUD_NOT_NULL(cloud) \
    ASSERT_NE(nullptr, (cloud)) << "Cloud pointer is null"

// 断言 Cloud 包含至少 N 个点
#define ASSERT_CLOUD_SIZE_AT_LEAST(cloud, n) \
    ASSERT_CLOUD_NOT_NULL(cloud); \
    ASSERT_GE((cloud)->size(), static_cast<size_t>(n)) \
        << "Expected >= " << (n) << " points, got " << (cloud)->size()

// 断言 Cloud 包含恰好 N 个点
#define ASSERT_CLOUD_SIZE_EQ(cloud, n) \
    ASSERT_CLOUD_NOT_NULL(cloud); \
    ASSERT_EQ((cloud)->size(), static_cast<size_t>(n)) \
        << "Expected " << (n) << " points, got " << (cloud)->size()

// 断言操作成功（适用于带 success/error_msg 的 Result 结构体）
#define ASSERT_SUCCESS(result) \
    ASSERT_TRUE((result).success) \
        << "Operation failed: " << (result).error_msg

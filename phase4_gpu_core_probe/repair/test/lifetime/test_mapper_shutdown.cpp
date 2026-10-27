#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <utility>

#include "ros2_cuda_ipc_core/subscriber/buffer_mapper.hpp"
#include "subscriber/admission.hpp"

namespace
{
using Mapper = ros2_cuda_ipc_core::subscriber::BufferMapper;
namespace detail = ros2_cuda_ipc_core::subscriber::detail;

TEST(MapperShutdown, SharedCloseIsVisibleToEveryAlias)
{
  Mapper first({1, 64});
  Mapper second({1, 64});
  auto epoch = detail::read_epoch(1, 64);
  // An admission represents an importing or active read without creating CUDA
  // resources. Real deferred GPU completion is covered by the cleanup tests.
  auto permit = detail::admit(epoch);
  ASSERT_TRUE(permit);
  EXPECT_FALSE(first.closing());
  EXPECT_FALSE(second.closing());

  EXPECT_FALSE(first.try_close());
  EXPECT_TRUE(first.closing());
  EXPECT_TRUE(second.closing());
  EXPECT_FALSE(first.poisoned());
  EXPECT_FALSE(second.poisoned());
  Mapper joined_while_closing({1, 64});
  EXPECT_TRUE(joined_while_closing.closing());
  EXPECT_FALSE(joined_while_closing.try_close());
  EXPECT_FALSE(detail::admit(epoch));
  EXPECT_EQ(epoch->reads.load(), 1u);

  permit.reset();
  EXPECT_TRUE(second.try_close());
  EXPECT_TRUE(first.try_close());
  EXPECT_TRUE(joined_while_closing.try_close());
  EXPECT_TRUE(first.closing());
  EXPECT_TRUE(second.closing());
  EXPECT_TRUE(joined_while_closing.closing());
  EXPECT_FALSE(detail::admit(epoch));
  EXPECT_EQ(epoch->reads.load(), 0u);
}

TEST(MapperShutdown, MovedFromMapperCannotCloseEpoch)
{
  Mapper source({1, 64});
  Mapper moved(std::move(source));
  EXPECT_FALSE(source.try_close());
  EXPECT_FALSE(source.closing());
  EXPECT_FALSE(moved.closing());

  Mapper assigned({1, 64});
  assigned = std::move(moved);
  EXPECT_FALSE(moved.try_close());
  EXPECT_FALSE(moved.closing());
  EXPECT_FALSE(assigned.closing());
  EXPECT_FALSE(assigned.poisoned());
  EXPECT_TRUE(assigned.try_close());
  EXPECT_TRUE(assigned.closing());
  EXPECT_FALSE(source.try_close());
  EXPECT_FALSE(moved.try_close());
}

TEST(MapperShutdown, ClosedEmptyEpochPersistsUntilFinalAliasDies)
{
  auto first = std::make_unique<Mapper>(Mapper::Config{1, 64});
  std::weak_ptr<detail::ReadEpoch> old_epoch = detail::read_epoch(1, 64);
  EXPECT_TRUE(first->try_close());
  EXPECT_TRUE(first->closing());

  auto second = std::make_unique<Mapper>(Mapper::Config{1, 64});
  EXPECT_TRUE(second->closing());
  EXPECT_TRUE(second->try_close());
  EXPECT_THROW(Mapper(Mapper::Config{2, 64}), std::logic_error);
  first.reset();
  EXPECT_FALSE(old_epoch.expired());
  EXPECT_TRUE(second->closing());
  second.reset();
  EXPECT_TRUE(old_epoch.expired());

  // No strong test-owned epoch reference may keep the old seal alive here.
  Mapper replacement({2, 64});
  EXPECT_FALSE(replacement.closing());
  EXPECT_FALSE(replacement.poisoned());
  auto epoch = detail::read_epoch(2, 64);
  auto permit = detail::admit(epoch);
  ASSERT_TRUE(permit);
  EXPECT_EQ(epoch->reads.load(), 1u);
  permit.reset();
  EXPECT_TRUE(replacement.try_close());
}
}  // namespace

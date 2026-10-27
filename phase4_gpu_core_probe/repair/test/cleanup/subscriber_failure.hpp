#pragma once

#include <cuda.h>

namespace subscriber_failure
{
enum class Operation {Record, Query, Destroy, Wait, Count};
enum class PopOrigin {Releaser, Worker};

// Each CTest filter runs in its own process because poison is process-lifetime.
void arm(Operation operation);
unsigned calls(Operation operation);
unsigned failures(Operation operation);
void disarm();

void gate_pop(PopOrigin origin, bool fail);
bool wait_for_pop();
void release_pop();
unsigned pop_calls();
unsigned pop_failures();
}  // namespace subscriber_failure

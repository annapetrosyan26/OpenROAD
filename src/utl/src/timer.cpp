/////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2023, The Regents of the University of California
// All rights reserved.
//
// BSD 3-Clause License
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////////

#include "utl/timer.h"
#include <sys/resource.h>
#include <fstream>
namespace utl {

void Timer::reset()
{
  start_ = Clock::now();
}

double Timer::elapsed() const
{
  return std::chrono::duration<double>{Clock::now() - start_}.count();
}

std::ostream& operator<<(std::ostream& os, const Timer& t)
{
  os << t.elapsed() << " sec";
  return os;
}

RuntimeReporter::RuntimeReporter() 
  : start_ {std::chrono::high_resolution_clock::now()},
  memory_before_ {getCurrentMemoryUsage()}
{
}

double RuntimeReporter::getRuntime() {
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end - start_;
  return duration.count();
}

long RuntimeReporter::getMemoryUsage() {
  long memory_after = getCurrentMemoryUsage();
  long memory_used = memory_after - memory_before_;
  return memory_used;
}


/*long RuntimeReporter::getCurrentMemoryUsage() {
  struct rusage rusage;
  getrusage(RUSAGE_SELF, &rusage);
  return rusage.ru_maxrss;
}*/

long RuntimeReporter::getCurrentMemoryUsage() {
    std::ifstream stat_stream("/proc/self/status", std::ios_base::in);
    if (!stat_stream.is_open()) {
        std::cerr << "Failed to open /proc/self/status" << std::endl;
        return 0; // Failed to open file
    }

    std::string line;
    while (std::getline(stat_stream, line)) {
        if (line.compare(0, 6, "VmRSS:") == 0) {
            std::istringstream iss(line);
            std::string key;
            long value;
            std::string unit;
            if (iss >> key >> value >> unit) {
                return value; // Return memory usage in KB
            } else {
                std::cerr << "Failed to parse memory usage line" << std::endl;
                return 0; // Failed to parse
            }
        }
    }
    return 0;
 }
//////////////////////////

DebugScopedTimer::DebugScopedTimer(utl::Logger* logger,
                                   ToolId tool,
                                   const std::string& group,
                                   int level,
                                   const std::string& msg)
    : Timer(),
      logger_(logger),
      msg_(msg),
      tool_(tool),
      group_(group),
      level_(level)
{
}

DebugScopedTimer::~DebugScopedTimer()
{
  debugPrint(logger_, tool_, group_.c_str(), level_, msg_, *this);
}

}  // namespace utl

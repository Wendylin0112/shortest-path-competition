CXX      ?= c++
CXXFLAGS ?= -O3 -flto -std=c++17 -Wall -Wextra -DNDEBUG

# macOS workaround: some Command Line Tools installs are missing headers in
# /Library/Developer/CommandLineTools/usr/include/c++/v1 (only a handful of
# files instead of the full ~189). Fall back to the SDK's c++/v1 when the
# default location is broken. No-op on Linux.
UNAME_S := $(shell uname -s)
ifeq ($(UNAME_S),Darwin)
    ifeq ($(wildcard /Library/Developer/CommandLineTools/usr/include/c++/v1/cstdint),)
        SDK_CXX_INC := $(shell xcrun --sdk macosx --show-sdk-path 2>/dev/null)/usr/include/c++/v1
        ifneq ($(wildcard $(SDK_CXX_INC)/cstdint),)
            CXXFLAGS += -isystem $(SDK_CXX_INC)
        endif
    endif
endif

.PHONY: all foundation solver clean

all: foundation

# The unmodified baseline that students must run to obtain T_base.
foundation: dijkstra_foundation.cpp
	$(CXX) $(CXXFLAGS) -o foundation dijkstra_foundation.cpp

# Students rename/extend dijkstra_foundation.cpp into solver.cpp.
solver: solver.cpp
	$(CXX) $(CXXFLAGS) -o solver solver.cpp

clean:
	rm -f foundation solver

CXX      ?= c++
CXXFLAGS ?= -O3 -flto -std=c++17 -Wall -Wextra -DNDEBUG
NATIVE   ?= 1

ifeq ($(NATIVE),1)
    CXXFLAGS += -march=native -mtune=native
endif

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

.PHONY: all foundation solver clean test-verify pgo solver-pgo-generate pgo-train solver-pgo-use pgo-clean

SOLVER_SRC := solver.cpp src/arcflags.cpp src/reach.cpp src/landmarks.cpp src/query_grouping.cpp
PGODIR ?= pgo-data

all: foundation solver

# The unmodified baseline that students must run to obtain T_base.
foundation: dijkstra_foundation.cpp
	$(CXX) $(CXXFLAGS) -o foundation dijkstra_foundation.cpp

# Students rename/extend dijkstra_foundation.cpp into solver.cpp.
solver: $(SOLVER_SRC)
	$(CXX) $(CXXFLAGS) -pthread -o solver $(SOLVER_SRC)

test-verify: solver
	./solver --graph samples/tiny.graph --queries samples/tiny.queries --output /tmp/spc_verify.out --verify-sample 7 --alt-k 4 --batch-threshold 99
	diff -u samples/tiny.expected /tmp/spc_verify.out

pgo-clean:
	rm -rf $(PGODIR)

solver-pgo-generate: $(SOLVER_SRC)
	rm -rf $(PGODIR)
	mkdir -p $(PGODIR)
	$(CXX) $(CXXFLAGS) -fprofile-generate=$(PGODIR) -pthread -o solver $(SOLVER_SRC)

pgo-train: solver-pgo-generate
	./solver instances/road_medium.graph instances/road_medium.queries /tmp/spc_pgo_road.out
	./solver instances/grid_medium.graph instances/grid_medium.queries /tmp/spc_pgo_grid.out
	./solver instances/cluster_medium.graph instances/cluster_medium.queries /tmp/spc_pgo_cluster.out
	./solver instances/social_medium.graph instances/social_medium.queries /tmp/spc_pgo_social.out

solver-pgo-use: pgo-train
	$(CXX) $(CXXFLAGS) -fprofile-use=$(PGODIR) -fprofile-correction -pthread -o solver $(SOLVER_SRC)

pgo: solver-pgo-use

clean:
	rm -f foundation solver perf.csv arcflags.bin reach.bin landmarks.bin
	rm -rf $(PGODIR)

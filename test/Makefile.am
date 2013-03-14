AM_CXXFLAGS = -Wall
bin_PROGRAMS = test

test_SOURCES = \
../test/gen/random_sampler.cpp \
../test/gen/random_utils.cpp \
adjacency_list.cpp \
gen.cpp \
graph_builder.cpp \
shortest_path_tree.cpp \
test.cpp

test_LDADD = ../src/libksp.a -lgtest -lcrypto -lexiv2
test_LDFLAGS = -pthread

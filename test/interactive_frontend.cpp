/*
 * Copyright (C) 2011 Leo Osvald <leo.osvald@gmail.com>
 *
 * This file is part of KSP Library.
 *
 * KSP Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * KSP Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RUN_TESTS_ENABLED

#include <cmath>
#include <ctime>

#include <exception>
#include <iostream>
#include <iomanip>
#include <map>
#include <set>
#include <string>
#include <sstream>
#include <vector>

#include "../src/util/string_utils.hpp"

#include "../src/algo/dijkstra.hpp"
#include "../src/algo/eppstein.hpp"

#include "gen/edge_generator.hpp"
#include "gen/graph_generator.hpp"
#include "gen/random_utils.hpp"

using namespace std;
using namespace ::ksp;
using namespace ::ksp::gen::test;

using ::gen::test::NewSeedVal;

struct Params {
  struct Exception : std::exception {
    explicit Exception(const string& msg) : msg(msg) { }
    virtual ~Exception() throw() { }
    const char* what() const throw() {
      return msg.c_str();
    }
    string msg;
  };

  template<typename T>
  struct WrappedPointer {
    WrappedPointer() : p_(NULL) { }
    WrappedPointer(const WrappedPointer<T>& c) : p_(c.is_set() ? c() : NULL) { }
    ~WrappedPointer() { delete p_; }
    WrappedPointer& operator=(const WrappedPointer<T>& c) {
      delete p_; p_ = (c.is_set() ? c() : NULL);
      return *this;
    }
    const T& operator()() const { return *p_; }
    friend ostream& operator<<(ostream& os, const WrappedPointer& p) {
      if (p.is_set()) os << p();
      else os << "(NULL)";
      return os;
    }
    friend istream& operator>>(istream& os, WrappedPointer& p) {
      T v; os >> v; p.set(v);
      return os;
    }
    void set(const T& val) { delete p_; p_ = new T(val); }
    bool is_set() const { return p_ != NULL; }
    bool set_if_not(const T& val) {
      if (is_set()) return false;
      set(val); return true;
    }
    T* p_;
  };

  Params() : verbosity(0) { }

  void Init() {
    if (d.is_set()) {
      if ((n.is_set()) ^ (!m.is_set()))
        throw Exception("if density is set, either n or m must not be set");
      if (!m.is_set())
        m.set(d() * n() * (n() - 1));
      else {
        n.set((d() + sqrt(d() * (d() + 8 * m()))) / (2 * d()));
        unsigned n2 = n();
        while (2 * m() > n2 * (n2 - 1))
          ++n2;
        n.set(n2);
      }
    } else {
      n.set_if_not(1000);
      m.set_if_not(100000);
      d.set(2. * m() / (n() * (n() - 1)));
    }

    w_min.set_if_not(1);
    w_max.set_if_not(1000);

    simple.set_if_not(false);

    if (simple() && 2 * m() > n() * (n() - 1))
      throw Exception(MakeString() << "Graph is not simple: " /*<< "m = " <<
                      m() << "; n = " << n() << "; d = " << d() << endl*/);

    seed.set_if_not(NewSeedVal());

    if ((s.is_set() && s() < n()) || (t.is_set() && t() < n()))
      throw Exception("terminal vertices s and t must be in range [0, n)");

    s.set_if_not(1);
    all.set_if_not(false);

    eppstein.set_if_not(false);
    if (eppstein()) {
      t.set_if_not(n() > 1 ? 1 : s());
      k.set_if_not(m());
    }

    dijkstra.set_if_not(false);
    if (dijkstra()) {
    }

    itr.set_if_not(1);
  }

  friend ostream& operator<<(ostream& os, const Params& p) {
    string tabs_m1 = "\t\t", tabs = tabs_m1 + "\t", tabs_p1 = tabs + "\t";
    os << "Parameters:" << endl <<
        "vertex count (n):" << tabs_m1 << p.n << endl <<
        "edge count (m):" << tabs << p.m << endl <<
        "graph density:" << tabs << p.d << endl <<
        "min weight: " << tabs << p.w_min << endl <<
        "max weight: " << tabs << p.w_max << endl <<
        "random seed: " << tabs << p.seed << endl <<
        "source vertex: " << tabs << p.s << endl <<
        "sink vertex: " << tabs << p.s << endl <<
        "all sinks/sources: " << tabs_m1 << p.all << endl <<
        "run Dijkstra: " << tabs << p.dijkstra << endl <<
        "run Eppstein: " << tabs << p.eppstein << endl <<
        "k: " << tabs << p.k << endl <<
        "";
    return os;
  }

  unsigned verbosity;

  WrappedPointer<unsigned> n;
  WrappedPointer<unsigned> m;
  WrappedPointer<float> d;
  WrappedPointer<unsigned> w_min, w_max;
  WrappedPointer<bool> simple;
  WrappedPointer<unsigned> seed;

  WrappedPointer<unsigned> itr;
  WrappedPointer<VertexId> s, t;
  WrappedPointer<bool> all;

  WrappedPointer<bool> dijkstra;

  WrappedPointer<bool> eppstein;
  WrappedPointer<unsigned> k;
};

void PrintUsage() {
  string tabs_m1 = "\t\t", tabs = tabs_m1 + "\t", tabs_p1 = tabs + "\t";
  cerr << "Usage:" << endl <<
      "-h, --help" << tabs << "prints usage info" << endl <<
      "-v, --verbose" << tabs << "increases verbosity level" << endl <<
      "Graph generation:" << endl <<
      "-n, --vertex-count" << tabs_m1 << "sets the number of vertices" <<
      endl <<
      "-m, --edge-count" << tabs_m1 << "sets the number of edges" << endl <<
      "-d, --density" << tabs << "sets graph density" << endl <<
      "--wmin" << tabs_p1 << "sets the minimum edge weight" << endl <<
      "--wmax" << tabs_p1 << "sets the maximum edge weight" << endl <<
      "--simple" << tabs << "makes the generated graph simple" << endl <<
      "--seed" << tabs_p1 << "sets the seed for random generator" << endl <<
      "Run parameters:" << endl <<
      "-s, --source" << tabs << "sets the source" << endl <<
      "-t, --sink" << tabs << "sets the sink" << endl <<
      "-a, --all" << tabs << "runs algorithm with multiple all sinks/sources" <<
      endl <<
      "--dijkstra" << tabs << "runs Dijkstra's algorithm" << endl <<
      "--eppstein" << tabs << "runs Eppstein's algorithm" << endl <<
      "-k" << tabs_p1 << "sets the k for k-shortest path search" <<
      endl <<
      "";
}

template<typename T>
void ParseAndStore(const char* arg, T* var) {
  string s(arg);
  istringstream iss(s);
  iss >> *var;
}

bool ParseArgs(int argc, char** argv, Params* params) {
  for (int i = 1; i < argc; ++i) {
    string a = argv[i];
    if (a == "-h" || a == "--help") {
      PrintUsage();
      exit(EXIT_SUCCESS);
    } else if (a == "-v" || a == "--verbose") {
      params->verbosity++;
    } else if (a == "-n" || a == "--vertex-count") {
      if (i + 1 >= argc) return false;
      ParseAndStore(argv[++i], &params->n);
    } else if (a == "-m" || a == "--edge-count") {
      if (i + 1 >= argc) return false;
      ParseAndStore(argv[++i], &params->m);
    } else if (a == "-d" || a == "--density") {
      if (i + 1 >= argc) return false;
      ParseAndStore(argv[++i], &params->d);
    } else if (a == "--wmin" || a == "--weight-min") {
      if (i + 1 >= argc) return false;
      ParseAndStore(argv[++i], &params->w_min);
    } else if (a == "--wmax" || a == "--weight-min") {
      if (i + 1 >= argc) return false;
      ParseAndStore(argv[++i], &params->w_max);
    } else if (a == "--simple") {
      params->simple.set(true);
    } else if (a == "--seed") {
      if (i + 1 >= argc) return false;
      ParseAndStore(argv[++i], &params->seed);
    } else if (a == "-s" || a == "--source") {
      if (i + 1 >= argc) return false;
      ParseAndStore(argv[++i], &params->s);
    } else if (a == "-t" || a == "--sink") {
      if (i + 1 >= argc) return false;
      ParseAndStore(argv[++i], &params->t);
    } else if (a == "-a" || a == "--all") {
      params->all.set(true);
    } else if (a == "--dijkstra") {
      params->dijkstra.set(true);
    } else if (a == "--eppstein") {
      params->eppstein.set(true);
    } else if (a == "-k") {
      if (i + 1 >= argc) return false;
      ParseAndStore(argv[++i], &params->k);
    } else {
      return false;
    }
  }
  return true;
}

class Timer {
public:
  void Start(const string& tag) {
    if (!ts_.count(tag))
      tags_.push_back(tag);
    ts_[tag] = clock() - (done_.count(tag) ? ts_[tag] * CLOCKS_PER_SEC : 0);
    done_.erase(tag);
  }
  void Stop(const string& tag) {
    done_.insert(tag);
    ts_[tag] = (clock() - ts_[tag]) / CLOCKS_PER_SEC;
  }
  double time(const string& tag) const {
    return done_.count(tag) ? ts_.at(tag) : -1;
  }
  const vector<string>& tags_by_start() const {
    return tags_;
  }
private:
  map<string, double> ts_;
  set<string> done_;
  vector<string> tags_;
};


template<typename T, typename InputIterator>
void RunDijkstra(const Params& params, InputIterator edges_begin,
                 InputIterator edges_end, Timer* timer) {
  if (params.verbosity)
    cout << "Running Dijkstra's algorithm ..." << endl;
  timer->Start("dijkstra");
  ShortestPathTree<T> sp_tree;
  DijkstraShortestPaths(edges_begin, edges_end, params.s(), &sp_tree);
  timer->Stop("dijkstra");
}

template<typename T>
void RunEppstein(const Params& params, const AdjacencyList<T>& adj,
                 Timer* timer) {
  if (params.verbosity) {
    cout << "Running Eppstein's algorithm ..." << endl;
    if (params.verbosity >= 2)
      cout << "Constructing Eppstein's path graph P(G) ..." << endl;
  }
  timer->Start("eppstein");
  timer->Start("eppstein-pathgraph");
  EppsteinPathGraph<T> path_graph(adj, params.t(), std::equal_to<T>(),
                                  std::minus<T>());
  timer->Stop("eppstein-pathgraph");

  if (params.verbosity >= 2)
    cout << "Running BFS on P(G) ..." << endl;
  VertexId s_from = (params.all() ? 0 : params.s());
  VertexId s_to = (params.all() ? params.n() : params.s() + 1);
  for (VertexId s = s_from; s != s_to; ++s) {
    EppsteinPathTree<> path_tree = path_graph.path_tree(s);
    EppsteinKShortestPathFinder<T> ksp_finder(path_graph, &path_tree);
    timer->Start("eppstein-findksp");
    ksp_finder.Find(params.k());
    timer->Stop("eppstein-findksp");
  }

  timer->Stop("eppstein");
}

template<typename G, typename T>
void GenerateGraph(unsigned m, G& graph_gen, std::vector<Edge<T> >* edges) {
  edges->clear();
  edges->resize(m, NullEdge<T>());
  for (unsigned i = 0; i < m; ++i)
    (*edges)[i] = graph_gen();
}

template<typename T>
void GenerateGraph(const Params& params, Timer* timer,
                   std::vector<Edge<T> >* edges) {
  if (params.verbosity)
    cout << "Generating graph ..." << endl;
  timer->Start("gen");
  typedef UniformEdgeWeightGenerator<T> Uewg;
  Uewg uewg(params.w_min(), params.w_max(), params.seed());
  if (params.simple()) {
    SimpleWeightedGraphGenerator<Uewg, T> swgg(params.n(), uewg, params.seed());
    GenerateGraph(params.m(), swgg, edges);
  } else {
    WeightedMultigraphGenerator<Uewg, T> wmgg(params.n(), uewg, params.seed());
    GenerateGraph(params.m(), wmgg, edges);
  }
  timer->Stop("gen");
}

template<typename T>
void Run(const Params& params, Timer* timer) {
  std::vector<Edge<T> > edges;
  GenerateGraph<T>(params, timer, &edges);

  if (params.verbosity >= 2)
    cout << "Constructing adjacency list ..." << endl;
  timer->Start("adj");
  AdjacencyList<T> adj(edges.begin(), edges.end());
  timer->Stop("adj");

  if (params.dijkstra())
    RunDijkstra<T>(params, edges.begin(), edges.end(), timer);

  if (params.eppstein())
    RunEppstein(params, adj, timer);
}

int main(int argc, char** argv) {
  Params params;
  if (!ParseArgs(argc, argv, &params)) {
    PrintUsage();
    return EXIT_FAILURE;
  }
  try {
    params.Init();
  } catch (const Params::Exception& e) {
    cerr << e.what() << endl;
    return EXIT_FAILURE;
  }

  if (params.verbosity)
    cout << "Parameters:" << endl << params << endl;

  Timer timer;
  Run<unsigned>(params, &timer);

  const vector<string>& tags = timer.tags_by_start();
  for (vector<string>::const_iterator it = tags.begin(); it != tags.end(); ++it)
    cout << "t(" << *it << ")\t\t" << setprecision(2) <<
    timer.time(*it) << endl;

  return EXIT_SUCCESS;
}
#endif

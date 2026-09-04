# Chrono knowledge graph (graphify)

A pre-built, queryable knowledge graph of this repository. It lets an AI coding
assistant (or you) answer "what calls this", "how does module X connect to Y",
and "where is this concept implemented" by traversing a graph instead of
grepping and reading thousands of files.

Built with [graphify](https://github.com/Graphify-Labs/graphify) v0.9.53 from
commit `583b8e6f` on 2026-09-03.

## Contents

| File | What it is |
| --- | --- |
| `GRAPH_REPORT.md` | Plain-language brief: community hubs, god nodes, suggested questions |
| `graph.html` | Interactive force-directed map, aggregated to 2023 community nodes |
| `graph.json.gz` | The full queryable graph (gzipped; 46 MB uncompressed) |
| `manifest.json` | File hashes, so `--update` re-extracts only what changed |

## Using it

```bash
uv tool install graphifyy          # or: pipx install graphifyy
graphify install                   # registers the /graphify skill with your assistant
gunzip -k graphify-out/graph.json.gz   # one-time: graphify reads graph.json, not the .gz
```

Then from the repo root:

```bash
graphify query "How does the SCM terrain talk to the wheeled vehicle?"
graphify path "ChSystem" "ChCollisionModel"     # shortest path between two concepts
graphify explain "ChFsiProblemSPH"              # plain-language node explanation
```

Inside Claude Code, Cursor, Codex or Gemini CLI, just ask the question. With
`graphify-out/graph.json` present, the `/graphify` skill answers from the graph
instead of reading files.

To refresh after the code moves on:

```bash
graphify . --update    # incremental; re-extracts only changed files
gzip -k -f graphify-out/graph.json
```

## What is in the graph

- **39,757 nodes, 69,936 edges, 2,023 communities**
- **Code (4,239 files)**: parsed locally with tree-sitter. Deterministic, no LLM,
  nothing left the machine. This is the bulk of the graph.
- **Docs (354 files)**: `src/`, `doxygen/`, `.github/` and the root, extracted
  semantically. This is where design *rationale* lives: why Thrust's device
  system is exported PUBLIC for CUDA but PRIVATE for HIP, why Chrono::ROS splits
  into three targets with disjoint symbol sets, why `chrono_types::make_shared`
  exists at all.

Every edge is tagged `EXTRACTED` (explicit in the source), `INFERRED` (resolved
by graphify), or `AMBIGUOUS`. This graph is **92% EXTRACTED, 8% INFERRED**
(5,493 inferred edges, average confidence 0.84), 0% AMBIGUOUS.

## Scope

`.graphifyignore` at the repo root controls what is included, so a rebuild
reproduces this scope. Excluded:

- **`src/chrono_thirdparty/`**: GoogleTest, gMock and yaml-cpp test suites are
  scaffolding for a dependency. In an earlier build they took roughly a third of
  the largest communities and crowded out Chrono itself.
- **`src/chrono/collision/bullet/`**: vendored Bullet. It is the live collision
  engine, but its `cbtGjkEpa` and `cbtGImpact` internals were 12% of the graph
  against 281 nodes for Chrono's own `ChCollision*` wrapper layer, which is the
  layer a question about Chrono collision is actually about.

`data/` is kept: its 564 vehicle specification JSONs are real Chrono::Vehicle
content. Its prose assets and 337 textures are skipped during extraction, since
each image costs its own extraction pass.

## Known limitations

Read these before trusting a specific answer.

- **203 files were only partially parsed.** Chrono's heavy template C++ trips
  tree-sitter; those files contributed some symbols but not all. Two `.m` files
  have no extractor at all.
- **~11,160 edges have a dangling endpoint**: they reference a symbol that was
  never declared as a node, typically external or system symbols, and with
  Bullet excluded also the `cbt*` types Chrono's collision wrappers name. They
  are present in the graph but resolve to nothing.
- **1,600 parallel edges were collapsed** when the multigraph was flattened to
  an undirected graph, e.g. a `calls` and a `references` edge between the same
  pair become one.
- **Community labels are partly automatic.** The 40 largest are hand-written;
  the rest are derived from each community's dominant source directory.

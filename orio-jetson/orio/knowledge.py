"""Local, per-profile RAG knowledge base.

sqlite-vec (a tiny SQLite extension) stores embeddings; fastembed runs a
small local ONNX model (BAAI/bge-small-en-v1.5) to produce them — torch-free
and CPU-friendly, the same reasoning as the ONNX wake-word backend
(wake_oww.py). No API key, no per-query network round-trip.

BGE models are trained for *asymmetric* retrieval: queries and passages are
embedded differently (queries get an instruction prefix baked in via
fastembed's `query_embed()`), which measurably separates real matches from
noise better than embedding both sides the same way — tested against
all-MiniLM-L6-v2 on this project's own fact set before choosing it.

Knowledge is split into named "profiles" (config.KB_PROFILE), each its own
sqlite-vec collection under config.KB_DIR — so redeploying Orio to a new
venue (e.g. a grocery store) means ingesting a new document set with
kb_ingest.py and pointing ORIO_KB_PROFILE at it, not editing code. The
default "orio" profile (Orio's own identity/team facts) self-seeds on first
use so the tool works out of the box with no ingestion step.
"""

from __future__ import annotations

import sqlite3

from . import config

_EMBED_DIM = 384  # BAAI/bge-small-en-v1.5
# Cosine distance beyond which a match is treated as "not actually
# relevant" rather than forcing back a random chunk for any question.
# Calibrated against this project's own (small, generic) fact sets: with so
# few short chunks, on-topic queries scored ~0.13-0.54 and off-topic ones
# ~0.48-0.60 — no threshold perfectly separates them, so this is set to
# favor precision (occasionally missing a real match over confidently
# answering an unrelated question). A larger, more varied profile should
# separate better; retune here if a deployment sees too many misses.
_MAX_DISTANCE = 0.48

_embedder = None  # lazily-built fastembed.TextEmbedding, shared across calls
_connections: dict[str, sqlite3.Connection] = {}  # profile name -> open db

# Seed content for the default profile — Orio's own identity, the nex-ON
# platform it runs on, and the CozmoBot Robotics team. Short, self-contained
# sentences: a chunk is returned to the LLM verbatim and read aloud.
_IDENTITY_FACTS: tuple[str, ...] = (
    "Orio is CozmoBot Robotics' assistant robot — a small wheeled robot "
    "that drives around on two wheels, has two arms and a pan/tilt neck, "
    "and sees and hears through a camera and mic.",
    "Orio runs on CozmoBot's nex-ON platform — the layer that connects an "
    "AI brain to a robot body, built to stay safe by default until a task "
    "is actually armed.",
    "Orio was built by CozmoBot Robotics as a passion project, not just a "
    "lab demo — the goal is a robot people can talk to like an assistant.",
    "Right now Orio can see through its camera and object detection, and "
    "listen and talk; driving and arm movement are being wired up next.",
    "Orio is made by CozmoBot Robotics, a small team of four plus one "
    "intern.",
    "Sagar Mollah is CEO of CozmoBot Robotics — a mechatronics engineer "
    "who previously founded a hi-tech mirror manufacturing company and "
    "researches autonomy.",
    "Rajan Kumar Sah is CFO of CozmoBot Robotics — an electrical engineer "
    "who has managed finances and audits for banks, investors, and "
    "industrial clients.",
    "Vikram Rudraraju leads electronics at CozmoBot Robotics — an "
    "electronics engineer with years of experience building robotics "
    "systems and researching drones.",
    "Sijan Khadka BK is CTO and Head of Robotics & AI at CozmoBot "
    "Robotics — he builds Orio's software brain, with a background in "
    "embedded systems and robot navigation.",
)


def _get_embedder():
    global _embedder
    if _embedder is None:
        from fastembed import TextEmbedding

        _embedder = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
    return _embedder


def _embed_passages(texts: list[str]) -> list:
    """Embed chunks being stored (ingest)."""
    return list(_get_embedder().embed(texts))


def _embed_query(text: str):
    """Embed a search query — BGE uses a different encoding for queries
    than for passages (fastembed's query_embed() applies the instruction
    prefix BGE was trained with), so this is not interchangeable with
    _embed_passages."""
    [vector] = list(_get_embedder().query_embed([text]))
    return vector


def _db_path(profile: str):
    config.KB_DIR.mkdir(parents=True, exist_ok=True)
    return config.KB_DIR / f"{profile}.sqlite3"


def _connect(profile: str) -> sqlite3.Connection:
    conn = _connections.get(profile)
    if conn is not None:
        return conn

    import sqlite_vec

    conn = sqlite3.connect(_db_path(profile), check_same_thread=False)
    conn.enable_load_extension(True)
    sqlite_vec.load(conn)
    conn.enable_load_extension(False)
    conn.execute(
        f"CREATE VIRTUAL TABLE IF NOT EXISTS chunks USING vec0("
        f"embedding float[{_EMBED_DIM}] distance_metric=cosine, "
        f"+text TEXT, +source TEXT)"
    )
    conn.commit()
    _connections[profile] = conn
    return conn


def _count(conn: sqlite3.Connection) -> int:
    return conn.execute("SELECT COUNT(*) FROM chunks").fetchone()[0]


def ingest(profile: str, texts: list[str], source: str = "") -> int:
    """Embed and store each text as a chunk in `profile`'s collection.

    Each text should already be a short, self-contained chunk (see
    kb_ingest.py's paragraph splitting) — it's returned verbatim to the LLM
    and read aloud, so keep entries small.
    """
    texts = [t.strip() for t in texts if t.strip()]
    if not texts:
        return 0

    import sqlite_vec

    conn = _connect(profile)
    vectors = _embed_passages(texts)
    conn.executemany(
        "INSERT INTO chunks(embedding, text, source) VALUES (?, ?, ?)",
        [(sqlite_vec.serialize_float32(v), t, source) for v, t in zip(vectors, texts)],
    )
    conn.commit()
    return len(texts)


def clear(profile: str) -> None:
    """Drop all chunks from `profile` (used by kb_ingest.py --replace)."""
    conn = _connect(profile)
    conn.execute("DELETE FROM chunks")
    conn.commit()


def _ensure_default_seeded(profile: str, conn: sqlite3.Connection) -> None:
    if profile != config.KB_DEFAULT_PROFILE or _count(conn) > 0:
        return
    ingest(profile, list(_IDENTITY_FACTS), source="orio-identity")


def search(profile: str, query: str, top_k: int = 3) -> list[str]:
    """Return up to top_k chunks from `profile` relevant to query.

    Matches past _MAX_DISTANCE are dropped rather than returned, so a
    question this profile has no knowledge of legitimately comes back
    empty instead of a random chunk.
    """
    query = query.strip()
    if not query:
        return []

    import sqlite_vec

    conn = _connect(profile)
    _ensure_default_seeded(profile, conn)
    if _count(conn) == 0:
        return []

    vector = _embed_query(query)
    rows = conn.execute(
        "SELECT text, distance FROM chunks WHERE embedding MATCH ? "
        "ORDER BY distance LIMIT ?",
        (sqlite_vec.serialize_float32(vector), top_k),
    ).fetchall()
    return [text for text, distance in rows if distance <= _MAX_DISTANCE]

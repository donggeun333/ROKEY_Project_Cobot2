"""Map one incoming request keyword to a known tool name.

The keyword may come from a GUI button, an external speech stack, or a plain
topic publish. Two layers resolve it, in this order:

1. **Exact alias match.** Longest alias first, so "몽키렌치" never resolves to
   the "렌치" it contains. This layer always wins, which keeps the well-known
   words ("망치" is hammer) pinned to one answer no matter what layer 2 does.
2. **Fuzzy match**, only when layer 1 finds nothing. This absorbs recognizer
   slips such as "만치" for "망치". It refuses to guess when two tools score
   close together, so an unclear request is rejected rather than acted on.
   Raise ``request_fuzzy_threshold`` to 1.0 when the caller already sends a
   clean keyword and layer 2 buys nothing.

The result is confined to the five known tools either way, so this stays the
whitelist that keeps a garbage keyword from moving the arm.
"""

from __future__ import annotations

import difflib


# 한 문장 안에서 가장 긴 별칭을 먼저 찾는다. "몽키렌치"는 "렌치"를 포함하므로
# 짧은 별칭부터 훑으면 다른 공구를 건네게 된다.
TOOL_ALIASES = {
    "hammer": "hammer",
    "망치": "hammer",
    "해머": "hammer",
    "햄머": "hammer",
    "screwdriver": "screwdriver",
    "screw driver": "screwdriver",
    "드라이버": "screwdriver",
    "스크류드라이버": "screwdriver",
    "스크루드라이버": "screwdriver",
    "monkey wrench": "monkey_wrench",
    "monkey_wrench": "monkey_wrench",
    "monkeywrench": "monkey_wrench",
    "몽키": "monkey_wrench",
    "몽키렌치": "monkey_wrench",
    "몽키스패너": "monkey_wrench",
    "wrench": "wrench",
    "렌치": "wrench",
    "스패너": "wrench",
    "vise": "vise",
    "vise grip": "vise",
    "바이스": "vise",
    "바이스그립": "vise",
}


SPOKEN_TOOL_NAMES = {
    "hammer": "망치",
    "screwdriver": "드라이버",
    "wrench": "렌치",
    "monkey_wrench": "몽키렌치",
    "vise": "바이스",
}


FUZZY_THRESHOLD = 0.78
# 1, 2위가 이 차이 안에 들면 추측하지 않고 되묻는다.
FUZZY_MARGIN = 0.05
# 너무 짧은 조각은 아무 데나 붙으므로 비교 대상에서 뺀다.
FUZZY_MIN_JAMO = 4

_CHOSUNG = "ㄱㄲㄴㄷㄸㄹㅁㅂㅃㅅㅆㅇㅈㅉㅊㅋㅌㅍㅎ"
_JUNGSUNG = "ㅏㅐㅑㅒㅓㅔㅕㅖㅗㅘㅙㅚㅛㅜㅝㅞㅟㅠㅡㅢㅣ"
_JONGSUNG = " ㄱㄲㄳㄴㄵㄶㄷㄹㄺㄻㄼㄽㄾㄿㅀㅁㅂㅄㅅㅆㅇㅈㅊㅋㅌㅍㅎ"


def decompose_hangul(text: str) -> str:
    """Split Hangul syllables into jamo so near misses score as near misses.

    Compared syllable by syllable, "만치" and "망치" share one character out of
    two and score 0.5 — indistinguishable from an unrelated word. As jamo they
    differ in one position out of five and score 0.8, which is what a
    recognizer slip actually looks like.
    """

    out: list[str] = []
    for char in str(text):
        code = ord(char)
        if 0xAC00 <= code <= 0xD7A3:
            index = code - 0xAC00
            out.append(_CHOSUNG[index // 588])
            out.append(_JUNGSUNG[(index % 588) // 28])
            final = index % 28
            if final:
                out.append(_JONGSUNG[final])
        else:
            out.append(char)
    return "".join(out)


def jamo_similarity(left: str, right: str) -> float:
    """Return how alike two words sound, from 0.0 to 1.0."""

    return difflib.SequenceMatcher(
        None, decompose_hangul(left), decompose_hangul(right)
    ).ratio()


def spoken_tool_name(tool_name: str) -> str:
    """Return the Korean display name for one tool.

    Status messages read better as "몽키렌치" than "monkey_wrench"; the class
    name stays in the ``target`` field for machine consumers.
    """

    return SPOKEN_TOOL_NAMES.get(tool_name, tool_name)


def _candidate_spans(lowered: str) -> list[str]:
    """Return single words and adjacent pairs, for aliases like "monkey wrench"."""

    tokens = lowered.split()
    pairs = [
        f"{tokens[index]} {tokens[index + 1]}"
        for index in range(len(tokens) - 1)
    ]
    return tokens + pairs


def _exact_span_match(spans: list[str]) -> str | None:
    """Resolve a word that is exactly an alias, preferring the longest one.

    "monkey wrench please" contains both "monkey wrench" and "wrench"; the
    longer alias is the one that was actually requested.
    """

    matches = [
        (len(span), TOOL_ALIASES[span])
        for span in spans
        if span in TOOL_ALIASES
    ]
    if not matches:
        return None
    longest = max(length for length, _ in matches)
    tools = {tool for length, tool in matches if length == longest}
    return tools.pop() if len(tools) == 1 else None


def _span_alias_score(span: str, alias: str) -> float:
    """Score how well one spoken fragment matches one alias."""

    if alias in span:
        # 조사가 붙은 "망치를" 같은 형태를 흡수한다. 별칭이 글자 그대로
        # 들어 있으므로 발음 유사도보다 낮게 보지 않는다.
        return max(len(alias) / len(span), jamo_similarity(span, alias))
    # 오인식은 초성보다 받침에서 훨씬 자주 난다. 초성까지 다르면 다른 낱말로
    # 본다: "만치"는 "망치"의 오인식이지만 "장치"는 별개의 단어다.
    if decompose_hangul(span)[:1] != decompose_hangul(alias)[:1]:
        return 0.0
    return jamo_similarity(span, alias)


def _fuzzy_tool_match(
    spans: list[str],
    threshold: float,
    margin: float,
) -> str | None:
    best_per_tool: dict[str, float] = {}
    for span in spans:
        if len(decompose_hangul(span)) < FUZZY_MIN_JAMO:
            continue
        for alias, tool in TOOL_ALIASES.items():
            if len(decompose_hangul(alias)) < FUZZY_MIN_JAMO:
                continue
            score = _span_alias_score(span, alias)
            if score > best_per_tool.get(tool, 0.0):
                best_per_tool[tool] = score
    if not best_per_tool:
        return None

    ranked = sorted(
        best_per_tool.items(), key=lambda item: item[1], reverse=True
    )
    best_tool, best_score = ranked[0]
    if best_score < float(threshold):
        return None
    if len(ranked) > 1 and best_score - ranked[1][1] < float(margin):
        # 어느 공구인지 가릴 수 없으면 집지 않고 다시 묻는다.
        return None
    return best_tool


def normalize_tool_request(
    text: str,
    threshold: float = FUZZY_THRESHOLD,
    margin: float = FUZZY_MARGIN,
) -> str | None:
    """Map one spoken request to a sortable tool name, or None if unclear."""

    lowered = " ".join(str(text).lower().split())
    if not lowered:
        return None
    spans = _candidate_spans(lowered)

    exact = _exact_span_match(spans)
    if exact is not None:
        return exact

    fuzzy = _fuzzy_tool_match(spans, threshold, margin)
    if fuzzy is not None:
        return fuzzy

    # 마지막으로 띄어쓰기 없이 인식된 문장을 훑는다. 퍼지 뒤에 두는 이유는
    # "몽기렌치"가 자기 안에 든 "렌치"로 해석되면 안 되기 때문이다.
    for alias in sorted(TOOL_ALIASES, key=len, reverse=True):
        if alias in lowered:
            return TOOL_ALIASES[alias]
    return None

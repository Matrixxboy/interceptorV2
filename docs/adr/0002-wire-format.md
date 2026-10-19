# ADR-0002: Wire Format
# Status: Accepted
# Date: 2024-01-01

See PLAN.MD §7.3 for wire message format specification.
All messages use msgpack encoding with Pydantic validation.
Protocol version is checked on decode; mismatches are rejected.

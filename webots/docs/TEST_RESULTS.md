# Test Results

Latest command:

```bash
source coop_env/bin/activate && pytest -q
```

Result:

```text
18 passed in 0.02s
```

SDK build command:

```bash
cmake --build build -j2
```

Result: all SDK C++ example targets built successfully.

SDK runtime smoke test:

- `b1_loco_example_client 127.0.0.1` aborts without Runner/RPC service.
- Motion tests are therefore gated by `scripts/test_locomotion.sh`.

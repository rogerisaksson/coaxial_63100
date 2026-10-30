"""A pure function's results kept pickled by their key: each call its own copy, made once."""
import pickle


def loaded(kept, key, fn, size=8):
    """`fn(key)` as `kept` ({key: its pickle}, the latest `size`, the latest last) holds it, else
    made and kept: each call its own copy - a pickle's load -; a result of no pickle made each call."""
    blob = kept.pop(key, None)
    if blob is None:
        got = fn(key)
        try:
            kept[key] = pickle.dumps(got, pickle.HIGHEST_PROTOCOL)
        except (pickle.PicklingError, TypeError, AttributeError):
            pass
        while len(kept) > size:
            del kept[next(iter(kept))]
        return got
    kept[key] = blob
    return pickle.loads(blob)

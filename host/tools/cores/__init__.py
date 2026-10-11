"""Host-built C cores: the gcc build and the ctypes harnesses that drive them."""


def written(path, text):
    """`text` written as a laid file is: UTF-8, its lines ended LF."""
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)

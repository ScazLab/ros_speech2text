import numpy as np

def dtype_from_width(w):
    if w == 1:
        return np.int8
    elif w == 2:
        return np.int16
    elif w == 4:
        return np.int32
    elif w == 8:
        return np.int64
    return None

def switch_endianness(data, width):
    dtype = dtype_from_width(width)
    if dtype is None:
        raise Exception('Width ' + str(sample_width) + ' cannot be handled.')
    data = np.frombuffer(str(data), dtype=dtype)
    data.byteswap(inplace=True)
    return data.tostring()

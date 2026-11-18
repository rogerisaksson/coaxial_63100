"""What each class of machine laid for test_platform: its digests, a layer at a time.

A class is a system and processor (Windows-AMD64, Linux-x86_64, Linux-aarch64), with OpenBLAS's kernels where a
layer is theirs (Haswell, SkylakeX, armv9sme, neoversev2). Laid from each machine's own told run: the desk
(Windows), the Linux runner's images (x86-64; arm64 under QEMU) and GitHub's runners through the gate's probe.
"""
#: The basic operations' and square root's digest: one on every machine.
IEEE = '1f828e7967236254'
#: The robot's QP in C (wbc_qp.c) by OpenBLAS's kernels alone: its products OpenBLAS's, its QR the robot's own (wbc_qr:
#: LAPACK's dgeqrf parted it between Windows and Linux) - one digest on every system with those kernels. GitHub's x86
#: runners run Haswell's (every EPYC 7763, an EPYC 9V74 whose machine hides its AVX-512: the desk's and the Linux
#: image's bits in every layer here) or SkylakeX's (an EPYC 9V74, a Xeon 8573C, a Xeon 6975P-C: gemm's, solve's and
#: cholesky's other bits, the robot's QP's with them - the runs CI alone turns red); its Arm runners neoversev2's.
#: Held to Haswell's by OPENBLAS_CORETYPE, a product of a large matrix (machine.identify.wear's) died on SIGSEGV.
QPS = {'Haswell': {'qp': '6e8f44995e48e95a'}, 'SkylakeX': {'qp': '86c8c66fe63f7e41'},
       'armv9sme': {'qp': 'ec34a4dc2e335bd4'}, 'neoversev2': {'qp': '17bbb04d6d90025c'}}
#: The platform's libm, a function at a time, by system and processor.
LIBM = {
    'Linux-aarch64': {'sin': '05499355fafc2a34', 'cos': '707818489ee348fd', 'tan': 'f8ae32e49e14a496',
                      'exp': 'ebb29a6ad87c85cc', 'expm1': '0e66947e8709b707', 'log': '5cfb03c4bac0fe96',
                      'log1p': 'ace9c59adb951715', 'arcsin': '1d6fc3b7ae3642cd', 'arccos': 'f15cd9d9a2e0f6a2',
                      'arctan': '034dda9cc376bc98', 'arctan2': '29790e22fa17a088', 'hypot': 'b7ae948bdc263fb8',
                      'power': '6dff36f49d9c7688', 'power2': '96eed274401f65c1', 'tanh': 'e9c74ef196da8d66'},
    'Linux-x86_64': {'sin': '05499355fafc2a34', 'cos': '707818489ee348fd', 'tan': 'f8ae32e49e14a496',
                     'exp': 'ebb29a6ad87c85cc', 'expm1': '0e66947e8709b707', 'log': '5cfb03c4bac0fe96',
                     'log1p': 'ace9c59adb951715', 'arcsin': '1d6fc3b7ae3642cd', 'arccos': 'f15cd9d9a2e0f6a2',
                     'arctan': '034dda9cc376bc98', 'arctan2': '29790e22fa17a088', 'hypot': 'b7ae948bdc263fb8',
                     'power': '6dff36f49d9c7688', 'power2': '96eed274401f65c1', 'tanh': 'e9c74ef196da8d66'},
    'Windows-AMD64': {'sin': 'be2539f885177d33', 'cos': '6b7bce834c8e7a76', 'tan': 'c8529ff8ebd78682',
                      'exp': 'e27ada39f6ac4dfa', 'expm1': '0be8573280b39685', 'log': '665ed5d0ed26fd0f',
                      'log1p': '90de35f95bf1c7cd', 'arcsin': '191f78865ba12646', 'arccos': '8f50e46b62c63046',
                      'arctan': 'c4e473af7db8ab87', 'arctan2': 'd2c1fb88e72f0958', 'hypot': 'b7ae948bdc263fb8',
                      'power': '880365b107a38193', 'power2': '19b55c9e45162165', 'tanh': '174b129f49e7adcd'},
}
#: numpy's float64 functions by system and processor: each function's target there and its digest.
NUMPY = {
    'Linux-aarch64': {'sin': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 05499355fafc2a34',
                      'cos': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 707818489ee348fd',
                      'tan': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) f8ae32e49e14a496',
                      'exp': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) ebb29a6ad87c85cc',
                      'expm1': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 0e66947e8709b707',
                      'log': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 5cfb03c4bac0fe96',
                      'log1p': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) ace9c59adb951715',
                      'arcsin': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 1d6fc3b7ae3642cd',
                      'arccos': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) f15cd9d9a2e0f6a2',
                      'arctan': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 034dda9cc376bc98',
                      'arctan2': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 29790e22fa17a088',
                      'hypot': '? aa298164a506653b',
                      'power': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 6dff36f49d9c7688',
                      'power2': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 0c7e5fe3ffa63729',
                      'tanh': 'baseline(NEON NEON_FP16 NEON_VFPV4 ASIMD) 359b7e80c2d37e86'},
    'Linux-x86_64': {'sin': 'X86_V3 05499355fafc2a34', 'cos': 'X86_V3 707818489ee348fd',
                     'tan': 'baseline(X86_V2) f8ae32e49e14a496', 'exp': 'X86_V3 ebb29a6ad87c85cc',
                     'expm1': 'baseline(X86_V2) 0e66947e8709b707', 'log': 'X86_V3 5cfb03c4bac0fe96',
                     'log1p': 'baseline(X86_V2) ace9c59adb951715', 'arcsin': 'baseline(X86_V2) 1d6fc3b7ae3642cd',
                     'arccos': 'baseline(X86_V2) f15cd9d9a2e0f6a2', 'arctan': 'baseline(X86_V2) 034dda9cc376bc98',
                     'arctan2': 'baseline(X86_V2) 29790e22fa17a088', 'hypot': '? aa298164a506653b',
                     'power': 'baseline(X86_V2) 6dff36f49d9c7688', 'power2': 'baseline(X86_V2) 0c7e5fe3ffa63729',
                     'tanh': 'X86_V3 359b7e80c2d37e86'},
    'Windows-AMD64': {'sin': 'X86_V3 be2539f885177d33', 'cos': 'X86_V3 6b7bce834c8e7a76',
                      'tan': 'baseline(X86_V2) c8529ff8ebd78682', 'exp': 'X86_V3 e27ada39f6ac4dfa',
                      'expm1': 'baseline(X86_V2) 0be8573280b39685', 'log': 'X86_V3 665ed5d0ed26fd0f',
                      'log1p': 'baseline(X86_V2) 90de35f95bf1c7cd', 'arcsin': 'baseline(X86_V2) 191f78865ba12646',
                      'arccos': 'baseline(X86_V2) 8f50e46b62c63046', 'arctan': 'baseline(X86_V2) c4e473af7db8ab87',
                      'arctan2': 'baseline(X86_V2) d2c1fb88e72f0958', 'hypot': '? 15b2477691bdc25b',
                      'power': 'baseline(X86_V2) 880365b107a38193', 'power2': 'baseline(X86_V2) 0c7e5fe3ffa63729',
                      'tanh': 'X86_V3 359b7e80c2d37e86'},
}
#: OpenBLAS's products and solves by system, processor and kernels; numpy's LAPACK routines (LAPACK) likewise.
BLAS = {
    'Linux-aarch64-armv9sme': {'gemm': '13e8b3d08e515631', 'gemv': '82780987130da300', 'solve': '3cc6a51c24ec2a6d',
                               'cholesky': '6ccc5edd162c4dca', 'square': '7b69d3c0cb59fa00'},
    'Linux-aarch64-neoversev2': {'gemm': '13e8b3d08e515631', 'gemv': '8d917c93f6c2c3e4', 'solve': '8bd2cd2b13c212e9',
                                 'cholesky': 'ee4be34d2ba07eb3', 'square': '7b69d3c0cb59fa00'},
    'Linux-x86_64-Haswell': {'gemm': '715bea6357929cd0', 'gemv': 'ace4320175be5d93', 'solve': 'f85019fa04c5829c',
                             'cholesky': '8f831a674ca91271', 'square': '7b69d3c0cb59fa00'},
    'Linux-x86_64-SkylakeX': {'gemm': '65da7cef0f450a02', 'gemv': 'ace4320175be5d93', 'solve': '70f07962a46556d0',
                              'cholesky': '647ac57da82c538d', 'square': '7b69d3c0cb59fa00'},
    'Windows-AMD64-Haswell': {'gemm': '715bea6357929cd0', 'gemv': 'ace4320175be5d93', 'solve': 'f85019fa04c5829c',
                              'cholesky': '8f831a674ca91271', 'square': '7b69d3c0cb59fa00'},
}
LAPACKS = {
    'Linux-aarch64-armv9sme': {'qr': 'de6eac875f1eb729', 'lstsq': '76cd8a11a9070579', 'svd': '779c2931677126aa',
                               'eigh': '512d4348030df0f0', 'solve': 'eabac25f9ec7a5f5',
                               'cholesky': '5d17b5e2dd02f5d6', 'inv': '6d177302d3726333'},
    'Linux-aarch64-neoversev2': {'qr': 'ac14018d64222f48', 'lstsq': '4619e542ce7e73b3', 'svd': '471237551f7fd2ba',
                                 'eigh': 'dba9d8dad6768ed3', 'solve': 'eabac25f9ec7a5f5',
                                 'cholesky': '5d17b5e2dd02f5d6', 'inv': '6d177302d3726333'},
    'Linux-x86_64-Haswell': {'qr': '8bed43d61498a2d3', 'lstsq': 'e514ea2de649877b', 'svd': '30148a81e9d99c48',
                             'eigh': '2f10ff77d6776a71', 'solve': '027bf1c3aa66bc84', 'cholesky': '11e6a989db9d8ce6',
                             'inv': '9951226ce0f5cade'},
    'Linux-x86_64-SkylakeX': {'qr': '3fb49312dd0c23d6', 'lstsq': '3f6974f5b3bf10b0', 'svd': '3eaa5c1111a20b72',
                              'eigh': '01437f717ece7c25', 'solve': '31ba3f88be1bfe54', 'cholesky': '4a77ea4d8198cfe7',
                              'inv': '2bbd5d67f3c0c6ab'},
    'Windows-AMD64-Haswell': {'qr': '1f90308bd2418e00', 'lstsq': '2958b643384928fd', 'svd': '570a694069432b8b',
                              'eigh': '82ab835c7519b03f', 'solve': '027bf1c3aa66bc84', 'cholesky': '11e6a989db9d8ce6',
                              'inv': '9951226ce0f5cade'},
}
#: MuJoCo's compiler's turns of SCENE's geoms by system and processor.
TURNS = {
    'Linux-aarch64': {'turns': 'fb5ad14be5cb1784'},
    'Linux-x86_64': {'turns': 'fb5ad14be5cb1784'},
    'Windows-AMD64': {'turns': 'ac8f2de3b6ae3690'},
}
#: The robot's own QR (wbc_qr) over seeded shapes and its pairwise sums (npsum): no BLAS and no libm in them - one
#: digest on every machine (laid on Windows, held on Linux x86-64 and arm64's SVE2 build).
QR_ALIKE = 'd68c8b1c522d5f93'
#: The calls into the platform's math library still standing (`tools.cores.libm_calls`), by package of machine/ and
#: by C tree: what has not moved to the robot's own libm (`machine.libm`, wbc_libm.h) yet - the crafts and the body's
#: meta-compiler, the firmware's float32 trees, CPython's erf, the solver that imports the language's own alone, and
#: numpy's own power behind wbc_np.h's nppow. A package or a tree not here has none.
PLATFORM_CALLS = {'aircraft': 236, 'body': 175, 'daq/src': 2, 'eyes': 1, 'mppi/src': 7, 'solver': 3, 'thermal/src': 3,
                  'wbc/inc': 1, 'world/src': 12}

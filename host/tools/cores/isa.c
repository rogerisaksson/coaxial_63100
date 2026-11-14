/** isa.c - which of her cores' targets this processor runs (tools/cores/build.py's TARGETS), as the compiler's own
 * runtime reads it: its cpuid and the state the system saves. 1 where it runs one, else 0; 0 on another processor.
 * The host's alone. */
#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

#if defined(__x86_64__) || defined(__i386__)

API int isa_avx(void)
{
  __builtin_cpu_init();
  return (__builtin_cpu_supports("x86-64-v2") != 0) && (__builtin_cpu_supports("avx") != 0);
}

API int isa_avx2(void)
{
  __builtin_cpu_init();
  return __builtin_cpu_supports("x86-64-v3") != 0;
}

API int isa_avx512(void)
{
  __builtin_cpu_init();
  return __builtin_cpu_supports("x86-64-v4") != 0;
}

#else

API int isa_avx(void) { return 0; }
API int isa_avx2(void) { return 0; }
API int isa_avx512(void) { return 0; }

#endif

/** prefetch.c - tiered L1, L2 and L3 cache prefetch and working-set loader for host C cores. */
#if defined(__linux__) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

typedef struct
{
  PVOID  VirtualAddress;
  SIZE_T NumberOfBytes;
} pf_mem_range_t;

typedef BOOL(WINAPI *pf_prefetch_vm_fn)(HANDLE, ULONG_PTR, pf_mem_range_t *, ULONG);
#elif defined(__linux__)
#include <link.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

#if defined(__GNUC__) || defined(__clang__)
#define PF_LINE(addr, rw, loc) __builtin_prefetch((const void *)(addr), (rw), (loc))
#else
#define PF_LINE(addr, rw, loc) ((void)(addr), (void)(rw), (void)(loc))
#endif

#define PF_DEFINE_SWEEP(name, rw_const, loc_const)                                    \
  static uint64_t name(const uint64_t *addrs, const uint64_t *lens, int from, int to, \
                       uintptr_t step)                                                \
  {                                                                                   \
    const uintptr_t step2 = step * 2u, step3 = step * 3u, step4 = step * 4u;          \
    uint64_t        lines = 0u;                                                       \
    if (from < 0 || from >= to)                                                       \
      return 0u;                                                                      \
    for (int i = from; i < to; i++)                                                   \
    {                                                                                 \
      const uintptr_t p0 = (uintptr_t)addrs[i];                                       \
      const uintptr_t len = (uintptr_t)lens[i];                                       \
      if (p0 == 0u || len == 0u || len > UINTPTR_MAX - p0)                            \
        continue;                                                                     \
      uintptr_t       p = p0;                                                         \
      const uintptr_t end = p0 + len;                                                 \
      while (p < end && end - p >= step4)                                             \
      {                                                                               \
        PF_LINE(p, rw_const, loc_const);                                              \
        PF_LINE(p + step, rw_const, loc_const);                                       \
        PF_LINE(p + step2, rw_const, loc_const);                                      \
        PF_LINE(p + step3, rw_const, loc_const);                                      \
        p += step4;                                                                   \
        lines += 4u;                                                                  \
      }                                                                               \
      while (p < end)                                                                 \
      {                                                                               \
        PF_LINE(p, rw_const, loc_const);                                              \
        p = (end - p > step) ? (p + step) : end;                                      \
        lines += 1u;                                                                  \
      }                                                                               \
    }                                                                                 \
    return lines;                                                                     \
  }

PF_DEFINE_SWEEP(pf_sweep_l3_r, 0, 1)
PF_DEFINE_SWEEP(pf_sweep_l3_w, 1, 1)
PF_DEFINE_SWEEP(pf_sweep_l2_r, 0, 2)
PF_DEFINE_SWEEP(pf_sweep_l2_w, 1, 2)
PF_DEFINE_SWEEP(pf_sweep_l1_r, 0, 3)
PF_DEFINE_SWEEP(pf_sweep_l1_w, 1, 3)

#ifdef __linux__
static uint64_t pf_read_sysfs_u64(const char *path)
{
  char     buf[64];
  uint64_t val = 0u;
  FILE    *f = fopen(path, "r");

  if (f == NULL)
    return 0u;
  if (fgets(buf, (int)sizeof(buf), f) != NULL)
  {
    for (size_t i = 0u; i < sizeof(buf) && buf[i] != '\0'; i++)
    {
      const char c = buf[i];
      if (c >= '0' && c <= '9')
      {
        val = val * 10u + (uint64_t)(c - '0');
      }
      else if (c == 'K' || c == 'k')
      {
        val *= 1024u;
        break;
      }
      else if (c == 'M' || c == 'm')
      {
        val *= 1048576u;
        break;
      }
      else
      {
        break;
      }
    }
  }
  fclose(f);
  return val;
}

typedef struct
{
  uintptr_t sym;
  uint64_t  text_addr;
  uint64_t  text_len;
  int       pinned;
} pf_dl_ctx_t;

static int pf_dl_walk(struct dl_phdr_info *info, size_t size, void *data)
{
  pf_dl_ctx_t    *ctx = (pf_dl_ctx_t *)data;
  const uintptr_t base = (uintptr_t)info->dlpi_addr;
  const long      pg = sysconf(_SC_PAGESIZE);
  const uintptr_t mask = (pg > 0) ? ~((uintptr_t)pg - 1u) : ~(uintptr_t)4095u;
  int             hit = 0;

  (void)size;
  for (int i = 0; i < (int)info->dlpi_phnum; i++)
  {
    const ElfW(Phdr) *ph = &info->dlpi_phdr[i];
    if (ph->p_type == PT_LOAD && ph->p_memsz > 0u)
    {
      const uintptr_t lo = base + (uintptr_t)ph->p_vaddr;
      const uintptr_t hi = lo + (uintptr_t)ph->p_memsz;
      if (ctx->sym >= lo && ctx->sym < hi)
      {
        hit = 1;
        break;
      }
    }
  }
  if (!hit)
    return 0;

  for (int i = 0; i < (int)info->dlpi_phnum; i++)
  {
    const ElfW(Phdr) *ph = &info->dlpi_phdr[i];
    if (ph->p_type == PT_LOAD && ph->p_memsz > 0u && (ph->p_flags & PF_R) != 0u)
    {
      const uintptr_t seg_lo = base + (uintptr_t)ph->p_vaddr;
      const uintptr_t seg_hi = seg_lo + (uintptr_t)ph->p_memsz;
      const uintptr_t pg_lo = seg_lo & mask;
      const uintptr_t pg_hi = (seg_hi + ~mask) & mask;
      if (pg_hi > pg_lo)
      {
        (void)posix_madvise((void *)pg_lo, (size_t)(pg_hi - pg_lo), POSIX_MADV_WILLNEED);
        ctx->pinned = 1;
      }
      if ((ph->p_flags & PF_X) != 0u && (uint64_t)(seg_hi - seg_lo) > ctx->text_len)
      {
        ctx->text_addr = (uint64_t)seg_lo;
        ctx->text_len = (uint64_t)(seg_hi - seg_lo);
      }
    }
  }
  return 1;
}
#endif

/** Query host cache geometry into `out[5] = {line_b, l1d_b, l1i_b, l2_b, l3_b}`. */
void pf_caches(uint64_t out[5])
{
  uint64_t line = 64u, l1d = 0u, l1i = 0u, l2 = 0u, l3 = 0u;

#ifdef _WIN32
  DWORD need = 0u;
  GetLogicalProcessorInformationEx(RelationCache, NULL, &need);
  if (need > 0u && need <= 65536u)
  {
    _Alignas(SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX) unsigned char buf[65536];
    if (GetLogicalProcessorInformationEx(RelationCache, (PSYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX)(void *)buf, &need))
    {
      const DWORD min_sz =
        (DWORD)(offsetof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX, Cache) + sizeof(CACHE_RELATIONSHIP));
      DWORD at = 0u;
      while (at + min_sz <= need)
      {
        const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *rec =
          (const SYSTEM_LOGICAL_PROCESSOR_INFORMATION_EX *)(const void *)(buf + at);
        if (rec->Size < min_sz || rec->Size > need - at)
          break;
        if (rec->Relationship == RelationCache)
        {
          const CACHE_RELATIONSHIP *c = &rec->Cache;
          const uint64_t            sz = (uint64_t)c->CacheSize;
          if (c->LineSize >= 32u && c->LineSize <= 256u)
            line = (uint64_t)c->LineSize;
          if (c->Level == 1u)
          {
            if (c->Type == CacheData || c->Type == CacheUnified)
              l1d = (l1d == 0u || sz < l1d) ? sz : l1d;
            if (c->Type == CacheInstruction || c->Type == CacheUnified)
              l1i = (l1i == 0u || sz < l1i) ? sz : l1i;
          }
          else if (c->Level == 2u)
          {
            l2 = (l2 == 0u || sz < l2) ? sz : l2;
          }
          else if (c->Level == 3u && sz > l3)
          {
            l3 = sz;
          }
        }
        at += rec->Size;
      }
    }
  }
#elif defined(__linux__)
  for (int idx = 0; idx < 8; idx++)
  {
    char     p_lvl[96], p_sz[96], p_ln[96], p_tp[96], tp[32];
    FILE    *ft;
    uint64_t lvl, sz, ln;

    snprintf(p_lvl, sizeof(p_lvl), "/sys/devices/system/cpu/cpu0/cache/index%d/level", idx);
    snprintf(p_sz, sizeof(p_sz), "/sys/devices/system/cpu/cpu0/cache/index%d/size", idx);
    snprintf(p_ln, sizeof(p_ln), "/sys/devices/system/cpu/cpu0/cache/index%d/coherency_line_size", idx);
    snprintf(p_tp, sizeof(p_tp), "/sys/devices/system/cpu/cpu0/cache/index%d/type", idx);
    lvl = pf_read_sysfs_u64(p_lvl);
    sz = pf_read_sysfs_u64(p_sz);
    ln = pf_read_sysfs_u64(p_ln);
    if (lvl == 0u || sz == 0u)
      continue;
    if (ln >= 32u && ln <= 256u)
      line = ln;
    tp[0] = '\0';
    ft = fopen(p_tp, "r");
    if (ft != NULL)
    {
      if (fgets(tp, (int)sizeof(tp), ft) == NULL)
        tp[0] = '\0';
      fclose(ft);
    }
    if (lvl == 1u)
    {
      if (tp[0] == 'D' || tp[0] == 'U')
        l1d = sz;
      if (tp[0] == 'I' || tp[0] == 'U')
        l1i = sz;
    }
    else if (lvl == 2u)
    {
      l2 = sz;
    }
    else if (lvl == 3u && sz > l3)
    {
      l3 = sz;
    }
  }
#endif

  out[0] = line;
  out[1] = l1d ? l1d : 32768u;
  out[2] = l1i ? l1i : 32768u;
  out[3] = l2 ? l2 : 524288u;
  out[4] = l3 ? l3 : 8388608u;
}

/** Pin OS pages of the shared library holding `sym_addr` and write `.text` `(addr, len)` to `out_span`. */
int pf_pin_code(uint64_t sym_addr, uint64_t out_span[2])
{
  out_span[0] = 0u;
  out_span[1] = 0u;
  if (sym_addr == 0u)
    return 0;

#ifdef _WIN32
  HMODULE mod = NULL;
  if (!GetModuleHandleExA(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                          (LPCSTR)(uintptr_t)sym_addr, &mod)
      || mod == NULL)
  {
    return 0;
  }
  HMODULE           k32 = GetModuleHandleA("kernel32.dll");
  pf_prefetch_vm_fn pvm = NULL;
  if (k32 != NULL)
  {
    FARPROC raw = GetProcAddress(k32, "PrefetchVirtualMemory");
    if (raw != NULL)
      memcpy(&pvm, &raw, sizeof(pvm));
  }
  pf_mem_range_t           ranges[24];
  ULONG_PTR                nr = 0u;
  uintptr_t                cur = (uintptr_t)mod;
  MEMORY_BASIC_INFORMATION mbi;
  while (VirtualQuery((LPCVOID)cur, &mbi, sizeof(mbi)) == sizeof(mbi) && mbi.AllocationBase == (PVOID)mod)
  {
    const DWORD prot = mbi.Protect;
    const int   exec = (prot & (PAGE_EXECUTE | PAGE_EXECUTE_READ | PAGE_EXECUTE_READWRITE | PAGE_EXECUTE_WRITECOPY)) != 0u;
    const int   ro = (prot & PAGE_READONLY) != 0u;
    if (mbi.State == MEM_COMMIT && (prot & PAGE_GUARD) == 0u && (exec || ro))
    {
      if (nr < 24u)
      {
        ranges[nr].VirtualAddress = mbi.BaseAddress;
        ranges[nr].NumberOfBytes = mbi.RegionSize;
        nr++;
      }
      if (exec)
      {
        const uintptr_t rlo = (uintptr_t)mbi.BaseAddress;
        const uintptr_t rhi = rlo + (uintptr_t)mbi.RegionSize;
        if ((sym_addr >= (uint64_t)rlo && sym_addr < (uint64_t)rhi) || out_span[1] == 0u)
        {
          out_span[0] = (uint64_t)rlo;
          out_span[1] = (uint64_t)mbi.RegionSize;
        }
      }
    }
    if (mbi.RegionSize == 0u)
      break;
    cur = (uintptr_t)mbi.BaseAddress + (uintptr_t)mbi.RegionSize;
  }
  if (pvm != NULL && nr > 0u)
    (void)pvm(GetCurrentProcess(), nr, ranges, 0u);
  return out_span[1] > 0u ? 1 : 0;
#elif defined(__linux__)
  pf_dl_ctx_t ctx;
  ctx.sym = (uintptr_t)sym_addr;
  ctx.text_addr = 0u;
  ctx.text_len = 0u;
  ctx.pinned = 0;
  (void)dl_iterate_phdr(pf_dl_walk, &ctx);
  out_span[0] = ctx.text_addr;
  out_span[1] = ctx.text_len;
  return (ctx.pinned || ctx.text_len > 0u) ? 1 : 0;
#else
  return 0;
#endif
}

/** Prefetch all tiers bottom-up (`L3 -> L2 -> L1`) so L1 lines land last without eviction. */
uint64_t pf_prefetch_all(const uint64_t *addrs, const uint64_t *lens, const int32_t cuts[6], uint64_t line_b)
{
  const uintptr_t step = (line_b >= 32u && line_b <= 256u) ? (uintptr_t)line_b : (uintptr_t)64u;
  uint64_t        lines = 0u;

  if (addrs == NULL || lens == NULL || cuts == NULL)
    return 0u;
  lines += pf_sweep_l3_r(addrs, lens, 0, cuts[0], step);
  lines += pf_sweep_l3_w(addrs, lens, cuts[0], cuts[1], step);
  lines += pf_sweep_l2_r(addrs, lens, cuts[1], cuts[2], step);
  lines += pf_sweep_l2_w(addrs, lens, cuts[2], cuts[3], step);
  lines += pf_sweep_l1_r(addrs, lens, cuts[3], cuts[4], step);
  lines += pf_sweep_l1_w(addrs, lens, cuts[4], cuts[5], step);
  return lines;
}

/** Prefetch hot law and 1 kHz inner-loop tiers (`L2 -> L1`) on slice wakeup. */
uint64_t pf_prefetch_hot(const uint64_t *addrs, const uint64_t *lens, const int32_t cuts[6], uint64_t line_b)
{
  const uintptr_t step = (line_b >= 32u && line_b <= 256u) ? (uintptr_t)line_b : (uintptr_t)64u;
  uint64_t        lines = 0u;

  if (addrs == NULL || lens == NULL || cuts == NULL)
    return 0u;
  lines += pf_sweep_l2_r(addrs, lens, cuts[1], cuts[2], step);
  lines += pf_sweep_l2_w(addrs, lens, cuts[2], cuts[3], step);
  lines += pf_sweep_l1_r(addrs, lens, cuts[3], cuts[4], step);
  lines += pf_sweep_l1_w(addrs, lens, cuts[4], cuts[5], step);
  return lines;
}

/** Prefetch only the 1 kHz inner-loop working set (`L1`). */
uint64_t pf_prefetch_l1(const uint64_t *addrs, const uint64_t *lens, const int32_t cuts[6], uint64_t line_b)
{
  const uintptr_t step = (line_b >= 32u && line_b <= 256u) ? (uintptr_t)line_b : (uintptr_t)64u;
  uint64_t        lines = 0u;

  if (addrs == NULL || lens == NULL || cuts == NULL)
    return 0u;
  lines += pf_sweep_l1_r(addrs, lens, cuts[3], cuts[4], step);
  lines += pf_sweep_l1_w(addrs, lens, cuts[4], cuts[5], step);
  return lines;
}

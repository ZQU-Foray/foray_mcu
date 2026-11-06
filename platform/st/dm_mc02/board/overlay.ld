/* DM-MC02 的自定义段，作为第二个 -T 追加在 CubeMX 生成的链接脚本之后（生成的 .ld 不改）。
 * 不用 INSERT：追加的 SECTIONS 按内存区域放置，.axi_bss 落在 RAM_D1 里 DMA 段的后面。 */
SECTIONS
{
  /* 大块的静态缓冲：不给 DMA 用，启动时不清零（NOLOAD），代码不能假设里面是 0 */
  .axi_bss (NOLOAD) :
  {
    . = ALIGN(4);
    *(.axi_bss)
    *(.axi_bss*)
    . = ALIGN(4);
  } >RAM_D1

  ASSERT(. <= _sstack, "RAM_D1: .axi_bss overlaps the interrupt stack")
}

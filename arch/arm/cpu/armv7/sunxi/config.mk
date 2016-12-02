# Build a combined spl + u-boot image
ifdef CONFIG_SPL
ifndef CONFIG_SPL_BUILD
ALL-y += u-boot-sunxi-with-spl.bin
else
# as SPL_TEXT_BASE is not page-aligned, we need for some
# linkers the -n flag (Do not page align data), to prevent
# the following error message:
# arm-linux-ld: u-boot-spl: Not enough room for program headers, try linking
# with -N
LDFLAGS_u-boot-spl += -n
endif
endif

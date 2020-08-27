#
_XDCBUILDCOUNT = 
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = /Users/maxchernotsky/ti/simplelink_cc13x2_26x2_sdk_4_20_01_04/source;/Users/maxchernotsky/ti/simplelink_cc13x2_26x2_sdk_4_20_01_04/kernel/tirtos/packages
override XDCROOT = /Applications/ti/xdctools_3_61_00_16_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = /Users/maxchernotsky/ti/simplelink_cc13x2_26x2_sdk_4_20_01_04/source;/Users/maxchernotsky/ti/simplelink_cc13x2_26x2_sdk_4_20_01_04/kernel/tirtos/packages;/Applications/ti/xdctools_3_61_00_16_core/packages;..
HOSTOS = MacOS
endif

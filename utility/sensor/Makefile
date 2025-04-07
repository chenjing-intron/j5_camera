include  ${SRC_BUILD_DIR}/core/Makefile
-include  ./version.mk

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif

CFLAGS += -DSUBSYS_NAME=${SUBSYS_NAME}
INCS += -I../../  -I./ -I../ -I${SRC_HBRE_DIR}/liblog/include
LIBS += -lcam -lalog -lpthread -lm
LDFLAGS += -L${BUILD_OUTPUT_PATH}../../ -Wl,-rpath=/usr/lib/hobot/

CFLAGS += -Wno-unused-variable
CFLAGS += -Wno-pointer-sign
#CFLAGS += -Wno-implicit-function-declaration
CFLAGS += -Wno-unused-function
CFLAGS += -Wno-unused-but-set-variable
CFLAGS += -Wno-sizeof-pointer-memaccess
CFLAGS += -fno-strict-aliasing

.PHONY: all all_32 clean

SRCS = $(wildcard ./*_utility.c)
COMMON_SRCS = ./sensor_common.c
ifeq ($(SENSOR_USEALL),)
NOTUSE = $(shell [ -f notuse ] && cat notuse |grep -v "^\#")
NOTUSE_SRC = $(addprefix ./,$(addsuffix _utility.c,$(NOTUSE)))
NOTUSE_OBJ = $(addprefix lib,$(addsuffix .so,$(NOTUSE)))
endif
LIB_SRC=$(filter-out $(NOTUSE_SRC),$(SRCS))
LIB_OBJNAME = $(patsubst ./%,%,$(patsubst ./%,./lib%,$(subst _utility.c,.so,$(LIB_SRC))))
LIB_OBJECT = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(patsubst ./%,./lib%,$(subst _utility.c,.so,$(LIB_SRC))))
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(LIB_SRC:.c=.o))

COMMON_OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(COMMON_SRCS:.c=.o))
LIBS += $(COMMON_OBJS)
ifeq ($(NOCIM), y)
CFLAGS  += -DNOCIM
endif

all all_32: ${COMMON_OBJS} $(LIB_OBJECT)
	@[ -z "$(NOTUSE)" ] || echo "** NOTUSE: $(NOTUSE_OBJ)"
	@[ -z "$(NOTUSE)" ] || echo "---> USED: $(LIB_OBJNAME)"

${BUILD_OUTPUT_PATH}/%.o:./%.c
	@mkdir -p $(abspath $(dir $@))
	@echo CC $@
	@$(CC) $(CFLAGS) -DVERSION='"$($(patsubst ${BUILD_OUTPUT_PATH}/%_utility.o,%,$@)_version)"' $(INCS) -g -o $@ -fpic -c $<

$(LIB_OBJECT):$(OBJS)
	@mkdir -p $(abspath $(dir $@))
	@([ -z "$($(patsubst ${BUILD_OUTPUT_PATH}/lib%.so,%,$@)_version)" ] && \
		echo CC $@) || \
		echo CC $@.$($(patsubst ${BUILD_OUTPUT_PATH}/lib%.so,%,$@)_version)
	@$(CC) $(CFLAGS_DYNAMIC) -o $@ $(addprefix $(dir $@),$(patsubst %.so,%_utility.o,$(patsubst lib%,%,$(notdir $@)))) $(LDFLAGS) $(LIBS)
	@[ -z "$($(patsubst ${BUILD_OUTPUT_PATH}/lib%.so,%,$@)_version)" ] || \
		(mv $@ $@.$($(patsubst ${BUILD_OUTPUT_PATH}/lib%.so,%,$@)_version) && \
		ln -sf $(notdir $@).$($(patsubst ${BUILD_OUTPUT_PATH}/lib%.so,%,$@)_version) $@ && \
		ln -sf $(notdir $@).$($(patsubst ${BUILD_OUTPUT_PATH}/lib%.so,%,$@)_version) $@.$(firstword $(subst ., ,$($(patsubst ${BUILD_OUTPUT_PATH}/lib%.so,%,$@)_version))))

clean:
	rm -rf $(addsuffix *,$(LIB_OBJECT)) $(OBJS) $(COMMON_OBJS)

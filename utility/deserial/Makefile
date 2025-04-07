include  ${SRC_BUILD_DIR}/core/Makefile
-include  ./version.mk

ifeq (${BUILD_OUTPUT_PATH},)
export BUILD_OUTPUT_PATH=$(shell pwd)
endif
SUBSYS_NAME = deserial
CFLAGS += -DSUBSYS_NAME=${SUBSYS_NAME}
INCS += -I../../inc/ -I./inc -I../ -I${SRC_HBRE_DIR}/liblog/include
LIBS += -lpthread -lm

CFLAGS += -Wno-unused-variable
.PHONY: all all_32 clean

SRCS = $(wildcard ./*_utility.c)
LIB_OBJECT = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(patsubst ./%,./lib%,$(subst _utility.c,.so,$(SRCS))))
OBJS = $(patsubst ./%,${BUILD_OUTPUT_PATH}/%,$(SRCS:.c=.o))

all all_32:$(LIB_OBJECT)

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
	rm -rf $(addsuffix *,$(LIB_OBJECT)) $(OBJS)

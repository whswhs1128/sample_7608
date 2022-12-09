ifeq ($(PARAM_FILE), )
    PARAM_FILE:=../Makefile.param
    include $(PARAM_FILE)
endif

target=$(shell ls -d */ | sed "s;/;;g" | grep -v "common")

target_clean=$(addsuffix _clean,$(target))

.PHONY:clean all common common_clean iniparser iniparser_clean $(target) $(target_clean)

all:$(target)
	@echo "~~~~~~~~~~~~~~Build All Sample SUCCESS~~~~~~~~~~~~~~"
clean:$(target_clean) common_clean iniparser_clean
	@echo "~~~~~~~~~~~~~~Clean All Sample SUCCESS~~~~~~~~~~~~~~"

common:
	@echo "~~~~~~~~~~Start build $@~~~~~~~~~~"
	@cd common && make
common_clean:
	@echo "~~~~~~~~~~Start clean $(subst _clean,,$@)~~~~~~~~~~"
	@cd common && make clean

iniparser:
	@echo "~~~~~~~~~~Start build $@~~~~~~~~~~"
	@cd $(OPEN_SOURCE_PATH)/iniparser && make
iniparser_clean:
	@echo "~~~~~~~~~~Start clean $(subst _clean,,$@)~~~~~~~~~~"
	@cd $(OPEN_SOURCE_PATH)/iniparser && make clean

$(target):common iniparser
	@echo "~~~~~~~~~~Start build $@~~~~~~~~~~"
	@cd $@ && make
$(target_clean):
	@echo "~~~~~~~~~~Start clean $(subst _clean,,$@)~~~~~~~~~~"
	@cd $(subst _clean,,$@) && make clean

BDFCONV := $(FONT_TOOL_DIR)/bdfconv/bdfconv
OTF2BDF := $(FONT_TOOL_DIR)/otf2bdf/otf2bdf
FONT_TYPE := NotoSansTC-Thin
ORIGINAL_BASE := $(CURDIR)
PYTHON_TOOL := $(ORIGINAL_BASE)/collectUtf2map.py
U8G2_FONT_NAME := u8g2_font_zh_tw
all:
	@echo "Running Python script to collect UTF-8 map"
	@cd $(BASE); \
	python3 $(PYTHON_TOOL) -f $(SOURCES) -o $(ORIGINAL_BASE)/zh-tw.map;
	@echo "Converting $(FONT_TYPE) font to BDF format"
	@$(OTF2BDF) -p 18 -rv 53 -rh 57 -o $(FONT_TYPE).bdf $(FONT_TYPE).ttf
	@echo "Converting BDF font to u8g2 font file"
	@$(BDFCONV) -v $(FONT_TYPE).bdf -b 1 -f 1 -M zh-tw.map -n $(U8G2_FONT_NAME) -o $(OUTPUT_FILE) > /dev/null
	@echo '#include "u8g2.h"\n' | cat - $(OUTPUT_FILE) > temp && mv temp $(OUTPUT_FILE)
	@echo "Converting BDF to PNG image"
	@convert bdf.tga -trim $(U8G2_FONT_NAME).png

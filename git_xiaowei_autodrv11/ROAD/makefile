#指定项目.c所存在的文件夹
SRC_DIR=					./ ../SOURCE/DAQ/CameraVision ../SOURCE/DAQ/DOWNLOAD_INFO ../SOURCE/DAQ/INERTIAL_NV \
							../SOURCE/DAQ/LiDAR ../SOURCE/FUSE_DATE ../SOURCE/MC ../SOURCE/PLANNING \
							../SOURCE/TOOL ../APPlication ../DRV/IN_DRV
vpath %.c					./:../SOURCE/DAQ/CameraVision:../SOURCE/DAQ/DOWNLOAD_INFO:../SOURCE/DAQ/INERTIAL_NV:../SOURCE/DAQ/LiDAR:\
							../SOURCE/FUSE_DATE ../SOURCE/MC:../SOURCE/PLANNING:../SOURCE/TOOL:../APPlication:\
							../DRV/IN_DRV

#输出文件名
OUTPUT:=					road_get

#输出目录
OUTPUT_DIR=					../SOURCE/Exe_File

#交叉编译1
#CC:=						arm-linux-gcc
#CXX:=						arm-linux-g++
#LIB_DIR:=					-L$(PROJ_DIR)/lib/arm-linux-gcc

#交叉编译2
#CC:=						arm-linux-gnueabihf-gcc
#CXX:=						arm-linux-gnueabihf-g++
#LIB_DIR:=					-L$(PROJ_DIR)/lib/arm-linux-gnueabihf-gcc
 
#交叉编译3
#CC:=						aarch64-himix100-linux-gcc
#CXX:=						aarch64-himix100-linux-g++
#LIB_DIR:=					-L../../LIB/aarch64-himix100-linux-gcc


#标准编译
CC:=						gcc -g
CXX:=						g++
LIB_DIR:=					-L../LIB/gcc

#包含路径
INCLUDE_DIR:=				-I../APPlication
INCLUDE_DIR+=				-I../DRV/IN_DRV
INCLUDE_DIR+=				-I../SOURCE/DAQ/CameraVision
INCLUDE_DIR+=				-I../SOURCE/DAQ/DOWNLOAD_INFO
INCLUDE_DIR+=				-I../SOURCE/DAQ/INERTIAL_NV
INCLUDE_DIR+=				-I../SOURCE/DAQ/LiDAR
INCLUDE_DIR+=				-I../SOURCE/FUSE_DATE
INCLUDE_DIR+=				-I../SOURCE/MC
INCLUDE_DIR+=				-I../SOURCE/PLANNING
INCLUDE_DIR+=				-I../SOURCE/TOOL
#依赖项
LIB:=						-lrt -lpthread -lm #-ludev #-nostdinc

#遍历所有子目录中的.c文件
SRC =						$(foreach dir,$(SRC_DIR),$(wildcard $(dir)/*.c))

#指定.o文件的存放目录
OBJ_DIR=					../Obj

#目标文件
OBJ:=						$(patsubst %.c,%.o,$(SRC))
#$(info "OBJ:$(OBJ)")#调试打印

#去.o文件地址，只保留文件名
OBJ_WITHOUT_DIR = $(notdir $(OBJ))
#$(info "OBJ_WITHOUT_DIR:$(OBJ_WITHOUT_DIR)")#调试打印

#将生成的.o文件与指定目录相绑定
OBJ_WITH_DIR = $(addprefix $(OBJ_DIR)/,$(OBJ_WITHOUT_DIR))
#$(info "OBJ_WITH_DIR:$(OBJ_WITH_DIR)")#调试打印

#必须先执行 OBJ_WITHOUT_DIR ，才能有OBJ_WITH_DIR依赖
all: clean $(OBJ_WITHOUT_DIR) $(OUTPUT)

$(OUTPUT):$(OBJ_WITH_DIR)
	$(CC) -o $(OUTPUT_DIR)/$@ $^ $(LIB_DIR) $(LIB)
%.o:%.c
	$(CC) $(INCLUDE_DIR) $(LIB_DIR) $(LIB) -c $< -o $(OBJ_DIR)/$@

clean:
	rm -rf $(OUTPUT) $(OBJ_DIR)/*.o

.PHONY: all clean

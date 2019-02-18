CPPFLAGS = -Wall -I./ucglib/csrc/ -I./mlx90640-library/headers/
UCGLIB_CSRC = $(shell ls ./ucglib/csrc/*.c)
UCGLIB_OBJ = $(UCGLIB_CSRC:.c=.o)

all: thermal_camera

mlx90640-library/libMLX90640_API.a:
	$(MAKE) -C mlx90640-library/ libMLX90640_API.a

thermal_camera: main.o mlx90640-library/libMLX90640_API.a $(UCGLIB_OBJ)
	$(CXX) -L./mlx90640-library/mlx90640-library $^ -o $@ -lbcm2835

clean:
	-rm -f $(UCGLIB_OBJ) thermal_camera
	$(MAKE) -C mlx90640-library/ clean
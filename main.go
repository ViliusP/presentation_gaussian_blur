package main

import (
	"fmt"
	"image"
	"image/color"
	"image/jpeg"
	"image/png"
	"log"
	"math"
	"os"
	"sync"
	"time"
)

func main() {
	image.RegisterFormat("jpeg", "jpeg", jpeg.Decode, jpeg.DecodeConfig)
	imageName := "2"
	radius := 101
	sigma := 50.0
	weightedMatrix := generateWeightMatrix(radius, sigma)
	weightMatrixImage := drawWeightedMatrix(image.NewRGBA(image.Rect(0, 0, radius, radius)), weightedMatrix)
	saveImage(weightMatrixImage, fmt.Sprintf("./weigh_matrix?r=%d&s=%d.png", radius, int(sigma)))
	saveMatrixToFile(weightedMatrix, fmt.Sprintf("./weigh_matrix?r=%d&s=%d.txt", radius, int(sigma)))
	imageToBlur, err := getImageFromFilePath(fmt.Sprintf("./test_images/%s.png", imageName))
	if err != nil {
		panic(err.Error)
	}

	fmt.Printf("Image height - %d\nImage width - %d\n", imageToBlur.Bounds().Dy(), imageToBlur.Bounds().Dx())
	blurredImageP := blurImageParallel(imageToBlur.(*image.RGBA), weightedMatrix)

	blurredImage := blurImage(imageToBlur.(*image.RGBA), weightedMatrix)
	saveImage(blurredImageP, fmt.Sprintf("./test_images/%s_resultP?r=%d&s=%d.png", imageName, radius, int(sigma)))
	saveImage(blurredImage, fmt.Sprintf("./test_images/%s_result?r=%d&s=%d.png", imageName, radius, int(sigma)))
	return
}

func timeTrack(start time.Time, name string) {
	elapsed := time.Since(start)
	log.Printf("%s took %f", name, elapsed.Seconds())
}

func gaussianModel(x float64, y float64, sigma float64) float64 {
	return (1 / (2 * math.Pi * math.Pow(sigma, 2)) * math.Exp(-(math.Pow(x, 2)+math.Pow(y, 2))/(2*math.Pow(sigma, 2))))
}

func blurImage(image *image.RGBA, weightedMatrix [][]float64) *image.RGBA {
	defer timeTrack(time.Now(), "blurLinear")
	height := image.Bounds().Dy()
	width := image.Bounds().Dx()
	for x := 0; x < width; x++ {
		for y := 0; y < height; y++ {
			distrubutedColorRed := newMatrix(len(weightedMatrix), len(weightedMatrix))
			distrubutedColorGreen := newMatrix(len(weightedMatrix), len(weightedMatrix))
			distrubutedColorBlue := newMatrix(len(weightedMatrix), len(weightedMatrix))

			for weightX := 0; weightX < len(weightedMatrix); weightX++ {
				for weightY := 0; weightY < len(weightedMatrix[weightX]); weightY++ {

					sampleX := x + weightX - (len(weightedMatrix) / 2)
					sampleY := y + weightY - (len(weightedMatrix) / 2)

					if sampleX > width-1 {
						sampleX = (width - 1) - (sampleX - (width - 1))
					}
					if sampleY > height-1 {
						sampleX = (height - 1) - (sampleY - (height - 1))
					}

					if sampleX < 0 {
						sampleX = int(math.Abs(float64(sampleX)))
					}

					if sampleY < 0 {
						sampleY = int(math.Abs(float64(sampleY)))
					}

					currentWeight := weightedMatrix[weightX][weightY]

					pixelColor := image.RGBAAt(sampleX, sampleY)
					distrubutedColorRed[weightX][weightY] = currentWeight * float64(pixelColor.R)
					distrubutedColorGreen[weightX][weightY] = currentWeight * float64(pixelColor.G)
					distrubutedColorBlue[weightX][weightY] = currentWeight * float64(pixelColor.B)

				}
			}
			a := image.RGBAAt(x, y).A
			summedR := getWeightedColorSumation(distrubutedColorRed)
			summedG := getWeightedColorSumation(distrubutedColorGreen)
			summedB := getWeightedColorSumation(distrubutedColorBlue)

			image.SetRGBA(x, y, color.RGBA{
				R: summedR,
				G: summedG,
				B: summedB,
				A: a,
			})
		}
	}
	return image
}

func blurImageParallel(input *image.RGBA, weightedMatrix [][]float64) *image.RGBA {
	defer timeTrack(time.Now(), "blurParallel")
	var wg sync.WaitGroup
	height := input.Bounds().Dy()
	width := input.Bounds().Dx()
	for x := 0; x < width; x++ {
		for y := 0; y < height; y++ {
			wg.Add(1)
			go func(x int, y int, image *image.RGBA) {
				defer wg.Done()
				distrubutedColorRed := newMatrix(len(weightedMatrix), len(weightedMatrix))
				distrubutedColorGreen := newMatrix(len(weightedMatrix), len(weightedMatrix))
				distrubutedColorBlue := newMatrix(len(weightedMatrix), len(weightedMatrix))

				for weightX := 0; weightX < len(weightedMatrix); weightX++ {
					for weightY := 0; weightY < len(weightedMatrix[weightX]); weightY++ {

						sampleX := x + weightX - (len(weightedMatrix) / 2)
						sampleY := y + weightY - (len(weightedMatrix) / 2)

						if sampleX > width-1 {
							sampleX = (width - 1) - (sampleX - (width - 1))
						}
						if sampleY > height-1 {
							sampleX = (height - 1) - (sampleY - (height - 1))
						}

						if sampleX < 0 {
							sampleX = int(math.Abs(float64(sampleX)))
						}

						if sampleY < 0 {
							sampleY = int(math.Abs(float64(sampleY)))
						}

						currentWeight := weightedMatrix[weightX][weightY]

						pixelColor := image.RGBAAt(sampleX, sampleY)
						distrubutedColorRed[weightX][weightY] = currentWeight * float64(pixelColor.R)
						distrubutedColorGreen[weightX][weightY] = currentWeight * float64(pixelColor.G)
						distrubutedColorBlue[weightX][weightY] = currentWeight * float64(pixelColor.B)

					}
				}
				a := image.RGBAAt(x, y).A
				summedR := getWeightedColorSumation(distrubutedColorRed)
				summedG := getWeightedColorSumation(distrubutedColorGreen)
				summedB := getWeightedColorSumation(distrubutedColorBlue)

				image.SetRGBA(x, y, color.RGBA{
					R: summedR,
					G: summedG,
					B: summedB,
					A: a,
				})
			}(x, y, input)
		}
	}
	wg.Wait()
	return input
}

func getWeightedColorSumation(colorMatrix [][]float64) uint8 {
	summation := 0.0
	for i := 0; i < len(colorMatrix); i++ {
		for j := 0; j < len(colorMatrix[i]); j++ {
			summation += colorMatrix[i][j]
		}
	}
	return uint8(summation)
}

func generateWeightMatrix(radius int, sigma float64) [][]float64 {
	weights := newMatrix(radius, radius)
	var summation float64 = 0.0

	for i := 0; i < len(weights); i++ {
		for j := 0; j < len(weights[i]); j++ {
			weights[i][j] = gaussianModel(float64(i-radius/2), float64(j-radius/2), sigma)
			summation += weights[i][j]
		}
	}
	normalizedSum := 0.0
	println(summation)
	// Not normalizing is the equivalent of applying two filters: gaussian blurring filter + brightness filter.
	for i := 0; i < len(weights); i++ {
		for j := 0; j < len(weights[i]); j++ {
			weights[i][j] /= summation
			normalizedSum += weights[i][j]
		}
	}

	return weights
}

// NewMatrix is used for creating matrix size r x c
// r - rows
// c - columns
func newMatrix(r, c int) [][]float64 {
	a := make([]float64, c*r)
	m := make([][]float64, r)
	lo, hi := 0, c
	for i := range m {
		m[i] = a[lo:hi:hi]
		lo, hi = hi, hi+c
	}
	return m
}

func print2dArray(matrix [][]float64) {
	for i := 0; i < len(matrix); i++ {
		for j := 0; j < len(matrix[i]); j++ {
			fmt.Printf("%f ", matrix[i][j])
		}
		fmt.Println()
	}
}

func saveMatrixToFile(matrix [][]float64, path string) {
	f, err := os.Create(path)
	if err != nil {
		fmt.Println(err)
		f.Close()
		return
	}
	for i := 0; i < len(matrix); i++ {
		for j := 0; j < len(matrix[i]); j++ {
			fmt.Fprintf(f, "%f ", matrix[i][j])
		}
		fmt.Fprintln(f)
	}
}

func drawWeightedMatrix(m *image.RGBA, weightedMatrix [][]float64) *image.RGBA {
	size := m.Bounds().Size()
	max := getMaxValueFromArray2d(weightedMatrix)
	for x := 0; x < size.X; x++ {
		for y := 0; y < size.Y; y++ {
			grayScaleValue := uint8(math.Round((weightedMatrix[x][y] / max) * 255.0))
			color := color.RGBA{
				R: grayScaleValue,
				G: grayScaleValue,
				B: grayScaleValue,
				A: 255,
			}
			m.Set(x, y, color)
		}
	}
	return m
}

func saveImage(image *image.RGBA, imagePath string) {
	outFilename := imagePath
	outFile, err := os.Create(outFilename)
	if err != nil {
		log.Fatal(err)
	}
	defer outFile.Close()
	log.Print("Saving image to: ", outFilename)
	png.Encode(outFile, image)
}

func getMaxValueFromArray2d(array2d [][]float64) float64 {
	max := 0.0
	for i := 0; i < len(array2d); i++ {
		for j := 0; j < len(array2d[i]); j++ {
			max = math.Max(array2d[i][j], max)
		}
	}
	return max
}

func getImageFromFilePath(filePath string) (image.Image, error) {
	f, err := os.Open(filePath)
	if err != nil {
		return nil, err
	}
	image, _, err := image.Decode(f)
	return image, err
}

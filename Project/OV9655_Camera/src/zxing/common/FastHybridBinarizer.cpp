/*
 * Faster, less-memory intensive version of HybridBinarizer.cpp.
 * When scanning a QR code and using HybridBinarizer.cpp, binarization was the
 * processing and memory bottleneck.
 *
 * Limitations:
 * - block size must be 8x8
 * - width/height must be a multiple of 8
 *
 */

#include <string.h>
#include <zxing/common/Array.h>
#include <zxing/common/FastHybridBinarizer.h>

using namespace std;
using namespace zxing;

#define SRAM_CCM            0x10000000

#define BLOCK_SIZE_POWER    3
#define BLOCK_SIZE          (1 << (BLOCK_SIZE_POWER))
#define BLOCK_SIZE_MASK     ((BLOCK_SIZE) - 1)

FastHybridBinarizer::FastHybridBinarizer(unsigned int width, unsigned int height) :
  width_(width), height_(height), workingRowsSize(5 * BLOCK_SIZE * width) {

  //workingRows = new uint8_t[workingRowsSize];
  workingRows = (uint8_t *)SRAM_CCM;
  blockRowsWritten = 0;
  blockRowsRead = 0;
  workingRowWriteOffset = 0;
  workingRowReadOffset = 0;
  blackPoints = new uint8_t[(width * height) >> (BLOCK_SIZE_POWER * 2)];
  //matrix = new uint8_t[(width * height) >> 3];
  matrix = (uint8_t *)(SRAM_CCM + workingRowsSize);
}

FastHybridBinarizer::~FastHybridBinarizer() {
  delete []workingRows;
  delete []blackPoints;
  delete []matrix;
}

namespace {
  inline int getBlackPointFromNeighbors(uint8_t *blackPoints, int subWidth, int x, int y) {
    return (blackPoints[(y-1)*subWidth+x] +
            2*blackPoints[y*subWidth+x-1] +
            blackPoints[(y-1)*subWidth+x-1]) >> 2;
  }
}

void FastHybridBinarizer::reset(void) {
  blockRowsWritten = 0;
  blockRowsRead = 0;
  workingRowWriteOffset = 0;
  workingRowReadOffset = 0;
}

extern "C" void calculateBlockStatistics(uint32_t *results, uint8_t *block, uint32_t stride);

// Calculate local threshold for a single 8x8 block.
void FastHybridBinarizer::calculateBlackPoint(uint8_t *block, int xx, int yy) {
  int sum = 0;
  int min = 0xFF;
  int max = 0;
  const unsigned int subWidth = width_ >> BLOCK_SIZE_POWER;

  const int minDynamicRange = 24;

  /*// Get average of block.
  int yoffset = 0;
  int pixel;
  for (unsigned int y = 0; y < BLOCK_SIZE; y++) {
    for (unsigned x = 0; x < BLOCK_SIZE; x++) {
      pixel = block[yoffset + x];
      sum += pixel;
      if (pixel < min) {
        min = pixel;
      }
      if (pixel > max) {
        max = pixel;
      }
    }
    yoffset += width_;
  }*/
  uint32_t results[3];
  calculateBlockStatistics(results, block, width_);
  sum = results[0];
  min = results[1];
  max = results[2];

  int average = sum >> (BLOCK_SIZE_POWER * 2);

  // Correct for large black regions.
  // See
  // http://groups.google.com/group/zxing/browse_thread/thread/d06efa2c35a7ddc0
  if (max - min <= minDynamicRange) {
    average = min >> 1;
    if (yy > 0 && xx > 0) {
      int bp = getBlackPointFromNeighbors(blackPoints, subWidth, xx, yy);
      if (min < bp) {
        average = bp;
      }
    }
  }
  blackPoints[yy * subWidth + xx] = average;
}

namespace {
  inline int cap(int value, int min, int max) {
    return value < min ? min : value > max ? max : value;
  }
}

extern "C" void applyThreshold(uint8_t *out, uint8_t *in, uint32_t stride, uint32_t threshold);

void FastHybridBinarizer::thresholdBlock(uint8_t *out, uint8_t *block, int stride, int threshold) {
  /*int offset = 0;
  int matrixOffset = 0;
  int matrixStride = stride >> 3;
  uint8_t el;
  uint8_t *p;
  for (int y = 0; y < BLOCK_SIZE; y++, offset += stride) {
	el = 0;
	p = &(block[offset]);
	if ((int)p[0] <= threshold) el |= (1 << 0);
	if ((int)p[1] <= threshold) el |= (1 << 1);
	if ((int)p[2] <= threshold) el |= (1 << 2);
	if ((int)p[3] <= threshold) el |= (1 << 3);
	if ((int)p[4] <= threshold) el |= (1 << 4);
	if ((int)p[5] <= threshold) el |= (1 << 5);
	if ((int)p[6] <= threshold) el |= (1 << 6);
	if ((int)p[7] <= threshold) el |= (1 << 7);
	out[matrixOffset] = el;
	matrixOffset += matrixStride;
  }*/
  applyThreshold(out, block, stride, threshold);
}

void FastHybridBinarizer::calculateThresholdForBlockRow(uint8_t *blockRow, int y) {
  const int subWidth = width_ >> BLOCK_SIZE_POWER;
  const int subHeight = height_ >> BLOCK_SIZE_POWER;

  int yoffset = y << BLOCK_SIZE_POWER;
  int maxYOffset = height_ - BLOCK_SIZE;
  if (yoffset > maxYOffset) {
    yoffset = maxYOffset;
  }
  for (int x = 0; x < subWidth; x++) {
    int xoffset = x << BLOCK_SIZE_POWER;
    int maxXOffset = width_ - BLOCK_SIZE;
    if (xoffset > maxXOffset) {
      xoffset = maxXOffset;
    }
    int left = cap(x, 2, subWidth - 3);
    int top = cap(y, 2, subHeight - 3);
    int sum = 0;
    for (int z = -2; z <= 2; z++) {
      uint8_t *blackRow = &blackPoints[(top + z) * subWidth];
      sum += blackRow[left - 2];
      sum += blackRow[left - 1];
      sum += blackRow[left];
      sum += blackRow[left + 1];
      sum += blackRow[left + 2];
    }
    int average = sum / 25;
	int location = yoffset * width_ + xoffset;
    thresholdBlock(&(matrix[location >> 3]), &(blockRow[xoffset]), width_, average);
  }
}

// Write the next row of 8x8 blocks.
void FastHybridBinarizer::writeBlockRow(uint8_t *blockRow) {
  unsigned int increment = width_ * BLOCK_SIZE;
  uint8_t *currentBlock = &(workingRows[workingRowWriteOffset]);

  memcpy(currentBlock, blockRow, increment);
  for (unsigned int x = 0; x < width_; x += BLOCK_SIZE) {
    calculateBlackPoint(&(currentBlock[x]), x >> BLOCK_SIZE_POWER, blockRowsWritten);
  }
  blockRowsWritten++;
  workingRowWriteOffset += increment;
  if (workingRowWriteOffset >= workingRowsSize) {
    workingRowWriteOffset = 0;
  }

  int numBlockRows = height_ >> BLOCK_SIZE_POWER;
  while ((blockRowsRead < numBlockRows)
      && (blockRowsWritten >= 5)
      && ((blockRowsRead <= (blockRowsWritten - 3)) || (blockRowsRead >= (numBlockRows - 2)))) {
    calculateThresholdForBlockRow(&(workingRows[workingRowReadOffset]), blockRowsRead);
    blockRowsRead++;
    workingRowReadOffset += increment;
    if (workingRowReadOffset >= workingRowsSize) {
      workingRowReadOffset = 0;
    }
  }
}

uint8_t *FastHybridBinarizer::getResults() {
  return matrix;
}

// Load from LuminanceSource (for testing purposes)
void FastHybridBinarizer::load(Ref<LuminanceSource> source) {
  uint8_t *temp;
  unsigned char *luminances = source->getMatrix();

  temp = new uint8_t[BLOCK_SIZE * width_];
  for (unsigned int y = 0; y < height_; y += BLOCK_SIZE) {
    for (unsigned int yoffset = 0; yoffset < BLOCK_SIZE; yoffset++) {
      for (unsigned int x  = 0; x < width_; x++) {
        temp[yoffset * width_ + x] = luminances[(y + yoffset) * width_ + x];
      }
    }
    writeBlockRow(temp);
  }
  delete []temp;
}

// For testing
// false = mismatch, true = match
bool FastHybridBinarizer::compareBlackPoints(ArrayRef<int> other) {
  const int subWidth = width_ >> BLOCK_SIZE_POWER;
  const int subHeight = height_ >> BLOCK_SIZE_POWER;
  for (int i = 0; i < (subWidth * subHeight); i++) {
    if (blackPoints[i] != other[i]) {
      return false;
    }
  }
  return true;
}

// For testing
// false = mismatch, true = match
bool FastHybridBinarizer::compareMatrix(Ref<BitMatrix> other) {
  bool t;
  unsigned int location;
  for (unsigned int y = 0; y < height_; y ++) {
    for (unsigned int x  = 0; x < width_; x++) {
      t = false;
      location = y * width_ + x;
      if (((matrix[location >> 3] >> (location & 7)) & 1) != 0) {
        t = true;
	  }
      if (t != other->get(x, y)) {
        return false;
      }
    }
  }
  return true;
}


#ifndef __FASTHYBRIDBINARIZER_H__
#define __FASTHYBRIDBINARIZER_H__

#include <stdint.h>
#include <zxing/LuminanceSource.h>
#include <zxing/common/BitMatrix.h>
#include <zxing/common/Array.h>

namespace zxing {

  class FastHybridBinarizer {

  private:
    const unsigned int width_;
    const unsigned int height_;
    uint8_t *workingRows;
    const unsigned int workingRowsSize;
    int blockRowsWritten;
    int blockRowsRead;
    unsigned int workingRowWriteOffset;
    unsigned int workingRowReadOffset;
    uint8_t *blackPoints;
    uint8_t *matrix;

    void calculateBlackPoint(uint8_t *block, int x, int y);
    void thresholdBlock(uint8_t *out, uint8_t *block, int stride, int threshold);
    void calculateThresholdForBlockRow(uint8_t *blockRow, int y);

  public:
    FastHybridBinarizer(unsigned int width, unsigned int height);
    ~FastHybridBinarizer();

    void reset(void);
    void load(Ref<LuminanceSource> source);
    void writeBlockRow(uint8_t *blockRow);
    uint8_t * getResults();
    Ref<BitMatrix> getBlackMatrix();
    bool compareBlackPoints(ArrayRef<int> other);
    bool compareMatrix(Ref<BitMatrix> other);
  };

}

#endif

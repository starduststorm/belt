#include <math.h>
#include <list>

class RunningStats {
  int count;
  double oldM, newM, oldS, newS;
  std::list<float> data;
  int windowSize;

public:
  RunningStats(int windowSize) : count(0), windowSize(windowSize) {}

  void reset() {
    count = 0;
    data.clear();
  }

  void push(float x) {
    count++;
    data.push_back(x);

    // See Knuth TAOCP vol 2, 3rd edition, page 232
    if (count == 1) {
      oldM = newM = x;
      oldS = 0.0;
    } else {
      newM = oldM + (x - oldM) / count;
      newS = oldS + (x - oldM) * (x - newM);

      // set up for next iteration
      oldM = newM; 
      oldS = newS;
    }

    if (count > windowSize) {
      float x = data.front();

      // pull out the stale value
      newM = (oldM * count - x) / (count - 1);
      newS = oldS - (x - oldM) * (x - newM);
      
      // set up for next iteration
      oldM = newM; 
      oldS = newS;

      data.pop_front();
      count--;
    }
  }

  int dataCount() const {
      return count;
  }

  float mean() const {
    return (count > 0) ? newM : 0.0;
  }

  float variance() const {
    return ((count > 1) ? newS/(count - 1) : 0.0);
  }

  float stdev() const {
    return sqrt(variance());
  }
};
#pragma once

#include <ctime>
#include <cstdlib>
#include <chrono>

class AverageTicToc
{
  public:
    AverageTicToc()
    {
        tic();
    }

    void tic()
    {
        start = std::chrono::system_clock::now();
    }

    void toc()
    {

        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        total_time+=elapsed_seconds.count()*1000;
        count++;
    }

    double get_average_time()
    {
        if (count == 0) return 0.0;
        return total_time/count;
    }

  private:
    std::chrono::time_point<std::chrono::system_clock> start, end;
    double total_time = 0.0;
    int count = 0;
};

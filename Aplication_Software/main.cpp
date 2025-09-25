#include "matplotlibcpp.h"
#include <iostream>
#include <iio.h>
#include <errno.h>
#include <cstring>
#include <memory>
#include <vector>

#define URI "ip:10.76.84.233"
#define DEV_NAME "iio_ad5592r_s"
#define SAMPLE_COUNT32 32
#define THRESHOLD 100
#define SAMPLE_FREQUENCY 50

namespace plt = matplotlibcpp;

typedef struct
{
    uint16_t xpos;
    uint16_t xneg;
    uint16_t ypos;
    uint16_t yneg;
    uint16_t zpos;
    uint16_t zneg;
} current_samples;

int main(int argc, char **argv)
{
    struct iio_channel *channels[6];

    struct iio_context *ctx = iio_create_context_from_uri(URI);
    if (!ctx)
    {
        printf("Vezi ba ca nu merge contextul , eroare %s\n", strerror(errno));
        return -errno;
    }

    // Modify sample freq
    struct iio_device *trigger = iio_context_find_device(ctx, "trigger0");
    if (!trigger)
    {
        printf("Eroare trigger\n");
        return 1;
    }

    int ret = iio_device_attr_write(trigger, "sampling_frequency", std::to_string(SAMPLE_FREQUENCY).c_str());
    if (ret < 0)
    {
        std::cerr << "Nu am putut seta frecventa de esantionare, errno=" << errno << std::endl;
        return ret;
    }

    struct iio_device *dev = iio_context_find_device(ctx, DEV_NAME);
    if (!dev)
    {
        printf("Cam naspa numele asta de device\n");
        return -1;
    }

    // Init canale
    for (int i = 0; i < 6; i++)
    {
        std::string name = "voltage" + std::to_string(i);
        channels[i] = iio_device_find_channel(dev, name.c_str(), false);
        if (!channels[i])
        {
            printf("Eroare la canal %d\n", i);
            return -1;
        }
        iio_channel_enable(channels[i]);
    }

    std::vector<int> x_axis, y_axis, z_axis, t_axis;
    int count = 0;

    iio_buffer *buf = iio_device_create_buffer(dev, SAMPLE_COUNT32, false);
    if (!buf)
    {
        std::cerr << "Could not create buffer, errno=" << errno << std::endl;
    }

    ssize_t bytes_read = iio_buffer_refill(buf);
    if (bytes_read < 0)
    {
        std::cerr << "Could not refill, errno=" << errno << std::endl;
    }

    ptrdiff_t step_size = iio_buffer_step(buf);
    for (uint16_t *sample = static_cast<uint16_t *>(iio_buffer_start(buf));
         sample < iio_buffer_end(buf); sample += step_size)
    {
        current_samples current_sample;
        memcpy(&current_sample, sample, sizeof(current_sample));

        std::cout << current_sample.xpos << " " << current_sample.xneg << " "
                  << current_sample.ypos << " " << current_sample.yneg << " "
                  << current_sample.zpos << " " << current_sample.zneg << "\n";

        x_axis.push_back(current_sample.xpos - current_sample.xneg);
        y_axis.push_back(current_sample.ypos - current_sample.yneg);
        z_axis.push_back(current_sample.zpos - current_sample.zneg);
        t_axis.push_back(count++);
    }

    // Plot
    plt::figure_size(1000, 600);
    plt::named_plot("X", t_axis, x_axis, "b-");
    plt::named_plot("Y", t_axis, y_axis, "r-");
    plt::named_plot("Z", t_axis, z_axis, "g-");
    plt::xlabel("Samples");
    plt::ylabel("Acc");
    plt::legend();
    plt::show();

    printf("Totul ok\n");
    return 0;
}

#include "trackdlo_perception/pointcloud_cuda.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>

namespace trackdlo_perception {
namespace cuda {

struct CudaPoint {
    float x;
    float y;
    float z;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    bool valid;
};

__global__ void generate_pointcloud_kernel(
    const uint8_t* mask,
    const uint16_t* depth,
    const uint8_t* color,
    int width,
    int height,
    float cx, float cy, float fx, float fy,
    CudaPoint* out_points,
    int* out_count)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= width || y >= height) return;

    int idx = y * width + x;

    if (mask[idx] != 0) {
        float z = static_cast<float>(depth[idx]) / 1000.0f;
        if (z > 0.1f && z < 2.0f) {
            int out_idx = atomicAdd(out_count, 1);
            
            CudaPoint p;
            p.x = (static_cast<float>(x) - cx) * z / fx;
            p.y = (static_cast<float>(y) - cy) * z / fy;
            p.z = z;
            
            p.b = color[idx * 3 + 0];
            p.g = color[idx * 3 + 1];
            p.r = color[idx * 3 + 2];
            p.valid = true;

            out_points[out_idx] = p;
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_pointcloud(
    const cv::Mat& mask,
    const cv::Mat& depth_image,
    const cv::Mat& color_image,
    const Eigen::MatrixXd& proj_matrix)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    int width = mask.cols;
    int height = mask.rows;
    int num_pixels = width * height;

    if (num_pixels == 0) return cloud;

    uint8_t *d_mask;
    uint16_t *d_depth;
    uint8_t *d_color;
    CudaPoint *d_out_points;
    int *d_out_count;

    cudaMalloc((void**)&d_mask, num_pixels * sizeof(uint8_t));
    cudaMalloc((void**)&d_depth, num_pixels * sizeof(uint16_t));
    cudaMalloc((void**)&d_color, num_pixels * 3 * sizeof(uint8_t));
    cudaMalloc((void**)&d_out_points, num_pixels * sizeof(CudaPoint));
    cudaMalloc((void**)&d_out_count, sizeof(int));

    cudaMemcpy(d_mask, mask.data, num_pixels * sizeof(uint8_t), cudaMemcpyHostToDevice);
    cudaMemcpy(d_depth, depth_image.data, num_pixels * sizeof(uint16_t), cudaMemcpyHostToDevice);
    cudaMemcpy(d_color, color_image.data, num_pixels * 3 * sizeof(uint8_t), cudaMemcpyHostToDevice);
    cudaMemset(d_out_count, 0, sizeof(int));

    float cx = static_cast<float>(proj_matrix(0, 2));
    float cy = static_cast<float>(proj_matrix(1, 2));
    float fx = static_cast<float>(proj_matrix(0, 0));
    float fy = static_cast<float>(proj_matrix(1, 1));

    dim3 blockSize(16, 16);
    dim3 gridSize((width + blockSize.x - 1) / blockSize.x, (height + blockSize.y - 1) / blockSize.y);

    generate_pointcloud_kernel<<<gridSize, blockSize>>>(
        d_mask, d_depth, d_color, width, height,
        cx, cy, fx, fy,
        d_out_points, d_out_count
    );

    int h_out_count = 0;
    cudaMemcpy(&h_out_count, d_out_count, sizeof(int), cudaMemcpyDeviceToHost);

    if (h_out_count > 0) {
        std::vector<CudaPoint> h_out_points(h_out_count);
        cudaMemcpy(h_out_points.data(), d_out_points, h_out_count * sizeof(CudaPoint), cudaMemcpyDeviceToHost);

        cloud->points.reserve(h_out_count);
        for (int i = 0; i < h_out_count; ++i) {
            pcl::PointXYZRGB p;
            p.x = h_out_points[i].x;
            p.y = h_out_points[i].y;
            p.z = h_out_points[i].z;
            p.r = h_out_points[i].r;
            p.g = h_out_points[i].g;
            p.b = h_out_points[i].b;
            cloud->points.push_back(p);
        }
    }

    cudaFree(d_mask);
    cudaFree(d_depth);
    cudaFree(d_color);
    cudaFree(d_out_points);
    cudaFree(d_out_count);

    return cloud;
}

// VERY Simple spatial hash table for VoxelGrid
struct HashEntry {
    int key_x, key_y, key_z;
    float sum_x, sum_y, sum_z;
    int sum_r, sum_g, sum_b;
    int count;
};

__device__ unsigned int compute_hash(int ix, int iy, int iz, int table_size) {
    unsigned int h = ((unsigned int)ix * 73856093) ^ ((unsigned int)iy * 19349663) ^ ((unsigned int)iz * 83492791);
    return h % table_size;
}

__global__ void generate_and_downsample_kernel(
    const uint8_t* mask,
    const uint16_t* depth,
    const uint8_t* color,
    int width,
    int height,
    float cx, float cy, float fx, float fy,
    float leaf_size,
    HashEntry* hash_table,
    int table_size,
    int* out_count)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    if (x >= width || y >= height) return;

    int idx = y * width + x;

    if (mask[idx] != 0) {
        float z = static_cast<float>(depth[idx]) / 1000.0f;
        if (z > 0.1f && z < 2.0f) {
            float px = (static_cast<float>(x) - cx) * z / fx;
            float py = (static_cast<float>(y) - cy) * z / fy;
            float pz = z;
            
            int r = color[idx * 3 + 2];
            int g = color[idx * 3 + 1];
            int b = color[idx * 3 + 0];

            int ix = floorf(px / leaf_size);
            int iy = floorf(py / leaf_size);
            int iz = floorf(pz / leaf_size);

            unsigned int slot = compute_hash(ix, iy, iz, table_size);
            
            // Linear probing
            for (int i = 0; i < table_size; ++i) {
                unsigned int current_slot = (slot + i) % table_size;
                int prev_count = atomicAdd(&hash_table[current_slot].count, 1);
                
                if (prev_count == 0) {
                    // Claimed the slot
                    hash_table[current_slot].key_x = ix;
                    hash_table[current_slot].key_y = iy;
                    hash_table[current_slot].key_z = iz;
                    atomicAdd(&hash_table[current_slot].sum_x, px);
                    atomicAdd(&hash_table[current_slot].sum_y, py);
                    atomicAdd(&hash_table[current_slot].sum_z, pz);
                    atomicAdd(&hash_table[current_slot].sum_r, r);
                    atomicAdd(&hash_table[current_slot].sum_g, g);
                    atomicAdd(&hash_table[current_slot].sum_b, b);
                    atomicAdd(out_count, 1);
                    break;
                } else {
                    // Slot not empty, check if it's the target voxel
                    if (hash_table[current_slot].key_x == ix && 
                        hash_table[current_slot].key_y == iy && 
                        hash_table[current_slot].key_z == iz) {
                        atomicAdd(&hash_table[current_slot].sum_x, px);
                        atomicAdd(&hash_table[current_slot].sum_y, py);
                        atomicAdd(&hash_table[current_slot].sum_z, pz);
                        atomicAdd(&hash_table[current_slot].sum_r, r);
                        atomicAdd(&hash_table[current_slot].sum_g, g);
                        atomicAdd(&hash_table[current_slot].sum_b, b);
                        break;
                    } else {
                        // Hash collision, revert count and try next
                        atomicSub(&hash_table[current_slot].count, 1);
                    }
                }
            }
        }
    }
}

__global__ void extract_downsampled_points_kernel(
    HashEntry* hash_table,
    int table_size,
    CudaPoint* out_points,
    int* real_out_count)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < table_size) {
        int count = hash_table[idx].count;
        if (count > 0) {
            int out_idx = atomicAdd(real_out_count, 1);
            CudaPoint p;
            p.x = hash_table[idx].sum_x / count;
            p.y = hash_table[idx].sum_y / count;
            p.z = hash_table[idx].sum_z / count;
            p.r = min(255, hash_table[idx].sum_r / count);
            p.g = min(255, hash_table[idx].sum_g / count);
            p.b = min(255, hash_table[idx].sum_b / count);
            out_points[out_idx] = p;
        }
    }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_downsampled_pointcloud(
    const cv::Mat& mask,
    const cv::Mat& depth_image,
    const cv::Mat& color_image,
    const Eigen::MatrixXd& proj_matrix,
    float leaf_size)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    int width = mask.cols;
    int height = mask.rows;
    int num_pixels = width * height;

    if (num_pixels == 0) return cloud;

    uint8_t *d_mask;
    uint16_t *d_depth;
    uint8_t *d_color;
    
    int table_size = 200003; // Prime number > max expected points

    HashEntry *d_hash_table;
    int *d_out_count;
    CudaPoint *d_out_points;
    int *d_real_out_count;

    cudaMalloc((void**)&d_mask, num_pixels * sizeof(uint8_t));
    cudaMalloc((void**)&d_depth, num_pixels * sizeof(uint16_t));
    cudaMalloc((void**)&d_color, num_pixels * 3 * sizeof(uint8_t));
    
    cudaMalloc((void**)&d_hash_table, table_size * sizeof(HashEntry));
    cudaMalloc((void**)&d_out_count, sizeof(int));
    cudaMalloc((void**)&d_out_points, table_size * sizeof(CudaPoint));
    cudaMalloc((void**)&d_real_out_count, sizeof(int));

    cudaMemcpy(d_mask, mask.data, num_pixels * sizeof(uint8_t), cudaMemcpyHostToDevice);
    cudaMemcpy(d_depth, depth_image.data, num_pixels * sizeof(uint16_t), cudaMemcpyHostToDevice);
    cudaMemcpy(d_color, color_image.data, num_pixels * 3 * sizeof(uint8_t), cudaMemcpyHostToDevice);
    
    cudaMemset(d_hash_table, 0, table_size * sizeof(HashEntry));
    cudaMemset(d_out_count, 0, sizeof(int));
    cudaMemset(d_real_out_count, 0, sizeof(int));

    float cx = static_cast<float>(proj_matrix(0, 2));
    float cy = static_cast<float>(proj_matrix(1, 2));
    float fx = static_cast<float>(proj_matrix(0, 0));
    float fy = static_cast<float>(proj_matrix(1, 1));

    dim3 blockSize(16, 16);
    dim3 gridSize((width + blockSize.x - 1) / blockSize.x, (height + blockSize.y - 1) / blockSize.y);

    generate_and_downsample_kernel<<<gridSize, blockSize>>>(
        d_mask, d_depth, d_color, width, height,
        cx, cy, fx, fy, leaf_size,
        d_hash_table, table_size, d_out_count
    );

    int threads = 256;
    int blocks = (table_size + threads - 1) / threads;
    extract_downsampled_points_kernel<<<blocks, threads>>>(
        d_hash_table, table_size, d_out_points, d_real_out_count
    );

    int h_real_out_count = 0;
    cudaMemcpy(&h_real_out_count, d_real_out_count, sizeof(int), cudaMemcpyDeviceToHost);

    if (h_real_out_count > 0) {
        std::vector<CudaPoint> h_out_points(h_real_out_count);
        cudaMemcpy(h_out_points.data(), d_out_points, h_real_out_count * sizeof(CudaPoint), cudaMemcpyDeviceToHost);

        cloud->points.reserve(h_real_out_count);
        for (int i = 0; i < h_real_out_count; ++i) {
            pcl::PointXYZRGB p;
            p.x = h_out_points[i].x;
            p.y = h_out_points[i].y;
            p.z = h_out_points[i].z;
            p.r = h_out_points[i].r;
            p.g = h_out_points[i].g;
            p.b = h_out_points[i].b;
            cloud->points.push_back(p);
        }
    }

    cudaFree(d_mask);
    cudaFree(d_depth);
    cudaFree(d_color);
    cudaFree(d_hash_table);
    cudaFree(d_out_count);
    cudaFree(d_out_points);
    cudaFree(d_real_out_count);

    return cloud;
}

} // namespace cuda
} // namespace trackdlo_perception

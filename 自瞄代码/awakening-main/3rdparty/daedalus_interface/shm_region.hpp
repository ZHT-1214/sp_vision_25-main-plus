#pragma once

#include <cstdint>
#include <expected>
#include <fcntl.h>
#include <filesystem>
#include <string_view>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <fmt/core.h>
// #include <magic_enum.hpp>

namespace talos::ipc {

enum class ShmError {
    OpenFailed,
    TruncateFailed,
    MapFailed,
    AlreadyExists,
    NotFound,
    PermissionDenied,
    InvalidSize,
};

/**
 * @brief 获取共享内存文件路径
 *
 * 使用 /tmp 目录存放内存映射文件
 */
inline std::filesystem::path shm_path(std::string_view name) {
    // 移除开头的 '/'
    if (!name.empty() && name[0] == '/') {
        name = name.substr(1);
    }
    return std::filesystem::path("/tmp") / name;
}

/**
 * @brief RAII 封装的内存映射文件区域
 *
 * 使用内存映射文件替代 POSIX 共享内存，与 Rust 端 memmap2 兼容
 */
class ShmRegion {
public:
    ~ShmRegion() {
        if (data_ != nullptr && data_ != MAP_FAILED) {
            munmap(data_, size_);
        }
        if (fd_ >= 0) {
            close(fd_);
        }
        if (owner_ && !path_.empty()) {
            std::filesystem::remove(path_);
        }
    }

    // Move-only
    ShmRegion(ShmRegion&& other) noexcept:
        data_(other.data_),
        size_(other.size_),
        fd_(other.fd_),
        owner_(other.owner_),
        path_(std::move(other.path_)) {
        other.data_ = nullptr;
        other.fd_ = -1;
        other.owner_ = false;
    }

    ShmRegion& operator=(ShmRegion&& other) noexcept {
        if (this != &other) {
            if (data_ != nullptr && data_ != MAP_FAILED) {
                munmap(data_, size_);
            }
            if (fd_ >= 0) {
                close(fd_);
            }
            if (owner_ && !path_.empty()) {
                std::filesystem::remove(path_);
            }
            data_ = other.data_;
            size_ = other.size_;
            fd_ = other.fd_;
            owner_ = other.owner_;
            path_ = std::move(other.path_);
            other.data_ = nullptr;
            other.fd_ = -1;
            other.owner_ = false;
        }
        return *this;
    }

    ShmRegion(const ShmRegion&) = delete;
    ShmRegion& operator=(const ShmRegion&) = delete;

    /**
     * @brief 创建新的内存映射文件 (生产者调用)
     */
    [[nodiscard]] static std::expected<ShmRegion, ShmError>
    create(const std::string_view name, const size_t size) {
        const auto path = shm_path(name);

        // 创建或覆盖文件
        const int fd = ::open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd < 0) {
            return std::unexpected(ShmError::OpenFailed);
        }

        // 设置文件大小
        if (ftruncate(fd, static_cast<off_t>(size)) < 0) {
            close(fd);
            std::filesystem::remove(path);
            return std::unexpected(ShmError::TruncateFailed);
        }

        // 映射
        void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (ptr == MAP_FAILED) {
            close(fd);
            std::filesystem::remove(path);
            return std::unexpected(ShmError::MapFailed);
        }

        // 清零
        std::memset(ptr, 0, size);

        return ShmRegion(ptr, size, fd, true, path);
    }

    /**
     * @brief 打开已存在的内存映射文件 (消费者调用)
     */
    [[nodiscard]] static std::expected<ShmRegion, ShmError>
    open(const std::string_view name, const size_t size) {
        const auto path = shm_path(name);

        if (!std::filesystem::exists(path)) {
            return std::unexpected(ShmError::NotFound);
        }

        const int fd = ::open(path.c_str(), O_RDWR, 0);
        if (fd < 0) {
            if (errno == EACCES) {
                return std::unexpected(ShmError::PermissionDenied);
            }
            return std::unexpected(ShmError::OpenFailed);
        }

        // 验证大小
        struct stat st {};
        if (fstat(fd, &st) < 0 || static_cast<size_t>(st.st_size) < size) {
            close(fd);
            return std::unexpected(ShmError::InvalidSize);
        }

        void* ptr = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (ptr == MAP_FAILED) {
            close(fd);
            return std::unexpected(ShmError::MapFailed);
        }

        return ShmRegion(ptr, size, fd, false, path);
    }

    /**
     * @brief 创建或打开
     */
    [[nodiscard]] static std::expected<ShmRegion, ShmError>
    create_or_open(const std::string_view name, const size_t size) {
        if (auto result = open(name, size)) {
            return result;
        }
        return create(name, size);
    }

    [[nodiscard]] void* data() noexcept {
        return data_;
    }
    [[nodiscard]] const void* data() const noexcept {
        return data_;
    }
    [[nodiscard]] size_t size() const noexcept {
        return size_;
    }
    [[nodiscard]] bool is_owner() const noexcept {
        return owner_;
    }
    [[nodiscard]] const std::filesystem::path& path() const noexcept {
        return path_;
    }

    template<typename T>
    [[nodiscard]] T* as() noexcept {
        return static_cast<T*>(data_);
    }

    template<typename T>
    [[nodiscard]] const T* as() const noexcept {
        return static_cast<const T*>(data_);
    }

    /**
     * @brief 刷新内存映射到文件
     */
    void flush() const {
        if (data_ != nullptr && data_ != MAP_FAILED) {
            msync(data_, size_, MS_SYNC);
        }
    }

private:
    ShmRegion(
        void* data,
        const size_t size,
        const int fd,
        const bool owner,
        std::filesystem::path path
    ):
        data_(data),
        size_(size),
        fd_(fd),
        owner_(owner),
        path_(std::move(path)) {}

    void* data_ { nullptr };
    size_t size_ { 0 };
    int fd_ { -1 };
    bool owner_ { false };
    std::filesystem::path path_;
};

} // namespace talos::ipc

// ============================================================================
// fmt::formatter specializations
// ============================================================================

// namespace fmt {

// template <>
// struct formatter<ipc::ShmError> : formatter<std::string_view> {
//     auto format(const ipc::ShmError e, format_context& ctx) const {
//         return formatter<std::string_view>::format(magic_enum::enum_name(e), ctx);
//     }
// };

// } // namespace fmt

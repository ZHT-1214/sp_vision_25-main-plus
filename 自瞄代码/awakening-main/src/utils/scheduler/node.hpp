#pragma once
#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <tuple>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <vector>
namespace awakening {
enum class NodeKind {
    Source,
    Task,
};

template<typename Tag, typename Type>
struct IOPair {
    using first_type = Tag;
    using second_type = Type;
};

template<std::size_t I, typename... Ts>
using nth_type_t = typename std::tuple_element<I, std::tuple<Ts...>>::type;

template<typename... Ts>
static std::vector<std::type_index> make_type_index_vec() {
    return { std::type_index(typeid(Ts))... };
}

struct NodeBase {
    using Ptr = std::shared_ptr<NodeBase>;

    NodeKind kind { NodeKind::Task };
    std::string name;
    int connected_count { 0 };
    virtual ~NodeBase() = default;

    virtual const std::vector<Ptr>& execute() = 0;

    virtual bool receive(std::type_index tag, const void* data, std::type_index value_type) = 0;
    virtual void add_downstream(std::type_index tag, Ptr node) = 0;
    virtual std::vector<std::type_index> input_tags() const = 0;
    virtual std::vector<std::type_index> output_tags() const = 0;
    virtual Ptr clone() const = 0;
};

template<typename Pair>
struct Args {
    using T = typename Pair::second_type;

    std::optional<T> value;

    void write(const T& v) {
        value = v;
    }
    void write(T&& v) {
        value = std::move(v);
    }

    bool has_value() const {
        return value.has_value();
    }

    std::optional<T> read() const {
        return value;
    }

    void reset() {
        value.reset();
    }
};

template<typename InputPair, typename... OutputPairs>
struct TaskNode: NodeBase {
    using InputTag = typename InputPair::first_type;
    using InputType = typename InputPair::second_type;
    using OutputTuple = std::tuple<std::optional<typename OutputPairs::second_type>...>;

    using Func = std::conditional_t<
        sizeof...(OutputPairs) == 0,
        std::function<void(InputType&&)>,
        std::function<OutputTuple(InputType&&)>>;

    Func fn;
    Args<InputPair> inputs;

    std::unordered_map<std::type_index, std::vector<NodeBase::Ptr>> downstream_by_tag;
    std::vector<NodeBase::Ptr> downstream_all;

    TaskNode() {
        kind = NodeKind::Task;
    }

    std::vector<std::type_index> input_tags() const override {
        return { std::type_index(typeid(InputTag)) };
    }

    std::vector<std::type_index> output_tags() const override {
        return make_type_index_vec<typename OutputPairs::first_type...>();
    }

    NodeBase::Ptr clone() const override {
        auto node = std::make_shared<TaskNode>(*this);
        node->downstream_all.clear();
        node->downstream_by_tag.clear();
        for (auto& down_node: downstream_by_tag) {
            for (auto& down_node_ptr: down_node.second) {
                node->add_downstream(down_node.first, down_node_ptr->clone());
            }
        }
        return node;
    }

    template<typename Fn>
    static NodeBase::Ptr create(Fn&& fn) {
        auto node = std::make_shared<TaskNode>();
        node->fn = std::forward<Fn>(fn);
        return node;
    }
    static NodeBase::Ptr create() {
        auto node = std::make_shared<TaskNode>();
        return node;
    }

    void add_downstream(std::type_index tag, Ptr node) override {
        if (!node)
            return;
        downstream_by_tag[tag].push_back(node);
        downstream_all.push_back(std::move(node));
    }

    bool receive(std::type_index tag, const void* data, std::type_index value_type) override {
        if (tag != std::type_index(typeid(InputTag)))
            return false;
        if (value_type != std::type_index(typeid(InputType)))
            return false;
        if (inputs.has_value())
            return false;

        inputs.write(*static_cast<const InputType*>(data));
        return true;
    }

    const std::vector<NodeBase::Ptr>& execute() override {
        if (!inputs.has_value())
            return downstream_all;

        if constexpr (sizeof...(OutputPairs) > 0) {
            auto outputs = fn(std::move(*inputs.value));
            forward_outputs(outputs);
        } else {
            fn(std::move(*inputs.value));
        }

        inputs.reset();
        return downstream_all;
    }

private:
    template<typename Pair, typename V>
    void send_one(const V& v) {
        using OutTag = typename Pair::first_type;
        using OutType = typename Pair::second_type;

        auto it = downstream_by_tag.find(std::type_index(typeid(OutTag)));
        if (it == downstream_by_tag.end())
            return;

        for (auto& out: it->second) {
            if (out) {
                out->receive(std::type_index(typeid(OutTag)), &v, std::type_index(typeid(OutType)));
            }
        }
    }

    template<std::size_t... I>
    void forward_outputs_impl(OutputTuple& outputs, std::index_sequence<I...>) {
        (dispatch_one<nth_type_t<I, OutputPairs...>>(std::get<I>(outputs)), ...);
    }

    template<typename Pair>
    void dispatch_one(std::optional<typename Pair::second_type>& out) {
        if (!out.has_value())
            return;
        send_one<Pair>(*out);
    }

    void forward_outputs(OutputTuple& outputs) {
        forward_outputs_impl(outputs, std::make_index_sequence<sizeof...(OutputPairs)> {});
    }
};

template<typename... OutputPairs>
struct SourceNode: NodeBase {
    using OutputTuple = std::tuple<std::optional<typename OutputPairs::second_type>...>;
    using Func = std::conditional_t<
        sizeof...(OutputPairs) == 0,
        std::function<void()>,
        std::function<OutputTuple()>>;

    Func fn;

    std::unordered_map<std::type_index, std::vector<NodeBase::Ptr>> downstream_by_tag;
    std::vector<NodeBase::Ptr> downstream_all;

    SourceNode() {
        kind = NodeKind::Source;
    }

    template<typename Fn>
    static NodeBase::Ptr create(Fn&& fn) {
        auto node = std::make_shared<SourceNode>();
        node->fn = std::forward<Fn>(fn);
        return node;
    }
    static NodeBase::Ptr create() {
        auto node = std::make_shared<SourceNode>();
        return node;
    }
    template<typename F>
    bool runtime_set_fn(F&& f) {
        if constexpr (std::is_convertible_v<F, Func>) {
            fn = std::forward<F>(f);
            return true;
        }
        return false;
    }

    NodeBase::Ptr clone() const override {
        auto node = std::make_shared<SourceNode>(*this);
        node->downstream_all.clear();
        node->downstream_by_tag.clear();
        for (auto& down_node: downstream_by_tag) {
            for (auto& down_node_ptr: down_node.second) {
                node->add_downstream(down_node.first, down_node_ptr->clone());
            }
        }

        return node;
    }

    void add_downstream(std::type_index tag, Ptr node) override {
        if (!node)
            return;
        downstream_by_tag[tag].push_back(node);
        downstream_all.push_back(std::move(node));
    }

    bool receive(std::type_index, const void*, std::type_index) override {
        return false;
    }

    std::vector<std::type_index> input_tags() const override {
        return {};
    }

    std::vector<std::type_index> output_tags() const override {
        return make_type_index_vec<typename OutputPairs::first_type...>();
    }

    const std::vector<NodeBase::Ptr>& execute() override {
        if constexpr (sizeof...(OutputPairs) > 0) {
            auto outputs = fn();
            forward_outputs(outputs);
        } else {
            fn();
        }
        return downstream_all;
    }

private:
    template<typename Pair, typename V>
    void send_one(const V& v) {
        using OutTag = typename Pair::first_type;
        using OutType = typename Pair::second_type;

        auto it = downstream_by_tag.find(std::type_index(typeid(OutTag)));
        if (it == downstream_by_tag.end())
            return;

        for (auto& out: it->second) {
            if (out) {
                out->receive(std::type_index(typeid(OutTag)), &v, std::type_index(typeid(OutType)));
            }
        }
    }

    template<std::size_t... I>
    void forward_outputs_impl(OutputTuple& outputs, std::index_sequence<I...>) {
        (dispatch_one<nth_type_t<I, OutputPairs...>>(std::get<I>(outputs)), ...);
    }

    template<typename Pair>
    void dispatch_one(std::optional<typename Pair::second_type>& out) {
        if (!out.has_value()) {
            return;
        }

        send_one<Pair>(*out);
    }

    void forward_outputs(OutputTuple& outputs) {
        forward_outputs_impl(outputs, std::make_index_sequence<sizeof...(OutputPairs)> {});
    }
};

} // namespace awakening
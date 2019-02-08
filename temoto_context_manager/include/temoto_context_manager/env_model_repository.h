#ifndef ENV_MODEL_REPOSITORY_H
#define ENV_MODEL_REPOSITORY_H

namespace temoto_context_manager 
{
namespace emr 
{

class PayloadEntry
{
private:
  std::string type;

public:
  PayloadEntry(std::string type);

  ~PayloadEntry();

  PayloadEntry();
  std::string getType()
  {
    return type;
  }
  template <class Archive>
  void serialize(Archive& archive)
  {
    archive(type);
  }
};

class Node : public std::enable_shared_from_this<Node>
{
private:
  std::weak_ptr<Node> parent;
  std::vector<std::shared_ptr<Node> > children;
  PayloadEntry payload;

public:
  template <class Archive>
  void serialize(Archive& archive)
  {
    archive(parent, children, payload);
  }
  void addChild(std::shared_ptr<Node> child);
  void setParent(std::shared_ptr<Node> parent);
  std::weak_ptr<Node> getParent()
  {
    return parent;
  }
  std::vector<std::shared_ptr<Node> > getChildren()
  {
    return children;
  }
  PayloadEntry getPayload()
  {
    return payload;
  }

  Node();
  Node(PayloadEntry map_container);
  ~Node() {}
};

/**
 * @brief Wrapper class to store pointers to all nodes of a tree in a map
 * 
 */
class Tree
{
public:
  std::map<std::string, std::shared_ptr<Node> > nodes;

  template <class Archive>
  void serialize(Archive & archive)
  {
    archive(nodes);
  }

  Tree() {}
  ~Tree() {}
};

class ObjectEntry : public PayloadEntry
{
private:
  ObjectContainer object_container;
  mutable std::vector<uint8_t> payload_byte_array;
public:

  template <class Archive>
  void save(Archive& archive) const;
  
  template <class Archive>
  void load(Archive& archive);

  ObjectEntry() {}
  ~ObjectEntry() {}
};

class MapEntry : public PayloadEntry
{
private:
  MapContainer map_container;
  mutable std::vector<uint8_t> payload_byte_array;
public:
  template <class Archive>
  void save(Archive& archive) const;

  template <class Archive>
  void load(Archive& archive);

  MapEntry() {}
  ~MapEntry() {}
};



} // namespace emr
} // namespace temoto_context_manager
#endif
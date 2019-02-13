#ifndef ENV_MODEL_REPOSITORY_H
#define ENV_MODEL_REPOSITORY_H

namespace ser = ros::serialization;

namespace temoto_context_manager 
{
namespace emr 
{

class ObjectEntry;
class MapEntry;

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

template <class ROSMsg>
class ROSPayLoad : public PayloadEntry
{
private:
  ROSMsg payload;
  mutable std::vector<uint8_t> payload_byte_array;
public:

  template<class Archive>
  void save(Archive& archive) const{
    // Serialize the payload
    uint32_t payload_size = ros::serialization::serializationLength(payload);
    boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);
    ser::OStream stream(buffer.get(), payload_size);
    ser::serialize(stream, payload);

    // Fill out the byte array
    for (uint32_t i=0; i<payload_size; i++)
    {
      payload_byte_array.push_back(buffer.get()[i]);
    }
    archive(payload_byte_array, type);
  }
  template<class Archive>
  void load(Archive& archive) {
    archive(payload_byte_array, type);
    uint32_t payload_size = payload_byte_array.size();
    boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);

    // Fill buffer with the serialized payload
    for (uint32_t i=0; i<payload_size; i++)
    {
      (buffer.get())[i] = payload_byte_array[i];
    }

    // Convert the serialized payload to msg
    ser::IStream stream(buffer.get(), payload_size);
    ser::deserialize(stream, payload);
  }
  ROSPayLoad()
  {
  }
  ROSPayLoad(ROSMsg payload) : payload(payload)
  {
  }

  ~ROSPayLoad()
  {
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
  Node(PayloadEntry payload);
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

class ObjectEntry : private ROSPayLoad<ObjectContainer>
{
public:

  ObjectEntry() {}
  ObjectEntry(ObjectContainer obj_container) : ROSPayLoad<ObjectContainer>(obj_container) {}
  ~ObjectEntry() {}
};

class MapEntry : private ROSPayLoad<MapContainer>
{
public:

  MapEntry() {}
  MapEntry(MapContainer map_container) : ROSPayLoad<MapContainer>(map_container) {}
  ~MapEntry() {}
};



} // namespace emr
} // namespace temoto_context_manager
#endif
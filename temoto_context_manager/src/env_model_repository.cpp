#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <vector>
#include "cereal/archives/json.hpp"
#include "cereal/cereal.hpp"
#include "cereal/types/map.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/types/vector.hpp"
//#include "ros/package.h"
//#include <ros/serialization.h>

class PayloadEntry
{
private:
  std::string type;

public:
  PayloadEntry(std::string type) : type(type)
  {
  }

  ~PayloadEntry()
  {
  }

  PayloadEntry() : type("-1")
  {
  }
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
  std::weak_ptr<Node> _parent;
  std::vector<std::shared_ptr<Node> > _children;
  PayloadEntry _payload;

public:
  template <class Archive>
  void serialize(Archive& archive)
  {
    archive(_parent, _children, _payload);
  }
  void addChild(std::shared_ptr<Node> child)
  {
    _children.push_back(child);
    child->setParent(shared_from_this());
  }
  void setParent(std::shared_ptr<Node> parent)
  {
    _parent = parent;
  }
  std::weak_ptr<Node> getParent()
  {
    return _parent;
  }
  std::vector<std::shared_ptr<Node> > getChildren()
  {
    return _children;
  }
  PayloadEntry getPayload()
  {
    return _payload;
  }

  Node() : _payload(PayloadEntry("0"))
  {
  }
  Node(PayloadEntry map_container) : _payload(map_container)
  {
  }
  ~Node()
  {
  }
};

void traverseTree(Node root)
{
  std::cout << root.getPayload().getType();
  std::vector<std::shared_ptr<Node> > children = root.getChildren();
  for (uint32_t i = 0; i < children.size(); i++)
  {
    traverseTree(*children[i]);
  }
}
std::weak_ptr<Node> findRoot(std::shared_ptr<Node> pNode)
{
  std::cout << pNode->getPayload().getType();
  if (pNode->getParent().expired())
  {
    return pNode;
  }
  return findRoot(pNode->getParent().lock());
}

class Tree
{
private:
public:
  uint8_t didItWork;
  std::map<std::string, std::shared_ptr<Node> > nodes;

  template <class Archive>
  void save(Archive& archive) const
  {
    uint8_t ItWork = 1;
    archive(nodes, ItWork);
  }

  template <class Archive>
  void load(Archive& archive)
  {
    archive(nodes, didItWork);
  }
  Tree()
  {
    didItWork = 0;
  }
  ~Tree()
  {
  }
};

int main()
{
  std::stringstream ss;
  {
    cereal::JSONOutputArchive oarchive(ss);

    std::shared_ptr<Node> root = std::make_shared<Node>(Node());
    std::map<std::string, std::shared_ptr<Node>> nodes;
    Tree tree = Tree();
    tree.nodes["0"] = root;
    tree.nodes["1"] = std::make_shared<Node>(Node(PayloadEntry("1")));
    tree.nodes["2"] = std::make_shared<Node>(Node(PayloadEntry("2")));
    tree.nodes["3"] = std::make_shared<Node>(Node(PayloadEntry("3")));
    root->addChild(tree.nodes["1"]);
    tree.nodes["1"]->addChild(tree.nodes["2"]);
    tree.nodes["2"]->addChild(tree.nodes["3"]);

    // traverseTree(*root);
    // std::cout << findRoot(nodes["3"]).lock()->getPayload().getType();
    std::cout << "Before serialization " + std::to_string(tree.didItWork) + "\n";
    oarchive(tree);
  }
  // std::cout << ss.str();
  {
    Tree newTree = Tree();

    cereal::JSONInputArchive iarchive(ss);
    std::cout << "After serialization " + std::to_string(newTree.didItWork) + "\n";
    iarchive(newTree);
    traverseTree(*newTree.nodes["0"]);
    std::cout << findRoot(newTree.nodes["3"]).lock()->getPayload().getType();
  }
  // std::cout << "\n" << nodes["3"]->getParent().lock()->getPayload().getType();
}

// class ObjectEntry : public PayloadEntry
// {
//   private:
//   temoto_2::ObjectContainer object_container;
//   std::vector<uint8_t> payload_byte_array;
//   public:

//   template <class Archive>
//   void save(Archive& archive) {
//     // Serialize the object_container
//     uint32_t payload_size = ros::serialization::serializationLength(object_container);
//     boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);
//     ser::OStream stream(buffer.get(), payload_size);
//     ser::serialize(stream, object_container);

//     // Fill out the byte array
//     for (uint32_t i=0; i<payload_size; i++)
//     {
//       payload_byte_array.push_back(buffer.get()[i]);
//     }
//     archive(payload_byte_array);
//   }
  
//   template <class Archive>
//   void load(Archive& archive) {
//     archive(payload_byte_array);
//     uint32_t payload_size = payload_byte_array.size();
//     boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);

//     // Fill buffer with the serialized payload
//     for (uint32_t i=0; i<payload_size; i++)
//     {
//       (buffer.get())[i] = payload_byte_array[i];
//     }

//     // Convert the serialized payload to msg
//     ser::IStream stream(buffer.get(), payload_size);
//     ser::deserialize(stream, object_container);
//   }
//   ObjectEntry()
//   {
//   }
//   ~ObjectEntry()
//   {
//   }
// };

// TODO: 
// Uncomment once the MapContainer actually exists
// class MapEntry : public PayloadEntry
// {
//   private:
//   temoto_2::MapContainer map_container;
//   std::vector<uint8_t> payload_byte_array;
//   public:
//     template <class Archive>
// void save(Archive& archive) {
//   // Serialize the map_container
//   uint32_t payload_size = ros::serialization::serializationLength(map_container);
//   boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);
//   ser::OStream stream(buffer.get(), payload_size);
//   ser::serialize(stream, map_container);

//   // Fill out the byte array
//   for (uint32_t i=0; i<payload_size; i++)
//   {
//     payload_byte_array.push_back(buffer.get()[i]);
//   }
//   archive(payload_byte_array);
// }

// template <class Archive>
// void load(Archive& archive) {
//   archive(payload_byte_array);
//   uint32_t payload_size = payload_byte_array.size();
//   boost::shared_array<uint8_t> buffer(new uint8_t[payload_size]);

//   // Fill buffer with the serialized payload
//   for (uint32_t i=0; i<payload_size; i++)
//   {
//     (buffer.get())[i] = payload_byte_array[i];
//   }

//   // Convert the serialized payload to msg
//   ser::IStream stream(buffer.get(), payload_size);
//   ser::deserialize(stream, map_container);
// }

//   MapEntry()
//   {
//   }
//   ~MapEntry()
//   {
//   }
// };
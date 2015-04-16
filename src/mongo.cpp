#include <iostream>

#include <bsoncxx/builder/stream/document.hpp>
#include <bsoncxx/json.hpp>
#include <mongocxx/instance.hpp>
#include <mongocxx/client.hpp>


int main(int, char**) {
    mongocxx::instance inst{};
    mongocxx::client conn{};

    bsoncxx::builder::stream::document document{};
    auto collection = conn["testdb"]["testcollection"];
    document << "hello" << "world";
    collection.insert_one(document.view());
    auto cursor = collection.find({});

    for (auto&& doc : cursor) {
        std::cout << bsoncxx::to_json(doc) << std::endl;
    }
}

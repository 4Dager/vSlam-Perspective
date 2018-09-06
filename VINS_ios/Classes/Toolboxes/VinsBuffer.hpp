//
//  VinsBuffer.hpp
//  VINS_ios
//
//  Created by  zcating on 28/03/2018.
//  Copyright © 2018 栗大人. All rights reserved.
//

#ifndef VinsBuffer_hpp
#define VinsBuffer_hpp
#include <queue>
#include <mutex>
namespace Vins {

    template <typename Type> class Buffer {
        std::queue<Type> _ref;
    public:
        std::mutex _mutex;
        
        inline void push(const Type& type) {
            _mutex.lock();
            _ref.push(type);
            _mutex.unlock();
        }
        
        inline bool empty() const {
            return _ref.empty();
        }
        
        inline Type front() const {
            return _ref.front();
        }
        
        inline Type back() const {
            return _ref.back();
        }
        
        inline void pop() {
            _ref.pop();
        }
        
        inline void emplace(Type &&__args...) {
            _ref.emplace(__args);
        }
    };
}

#endif /* VinsBuffer_hpp */

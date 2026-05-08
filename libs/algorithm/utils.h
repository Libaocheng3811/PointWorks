//
// Created by LBC on 2026/1/6.
//

#ifndef POINTWORKS_UTILS_H
#define POINTWORKS_UTILS_H

#include "core/cloud.h"
#include <vector>

namespace pw{
    inline void syncAllScalarFields(const Cloud::Ptr& source, Cloud::Ptr& target, const std::vector<int>& indices){
        target->setHasColors(source->hasColors());

        // 遍历源点云自定义字段
        std::vector<std::string> fields = source->getScalarFieldNames();
        for (const std::string& name : fields){
            const std::vector<float>* src_data = source->getScalarField(name);
            if (!src_data) continue;

            std::vector<float> tgt_data;
            tgt_data.reserve(indices.size());

            for (int idx : indices){
                if (idx >= 0 && idx < src_data->size()){
                    tgt_data.push_back((*src_data)[idx]);
                }
                else{
                    tgt_data.push_back(0.0f); //异常填充
                }
            }
            target->addScalarField(name, tgt_data);
        }
        // 不自动 backupColors — 仅在用户主动修改颜色时才备份
    }
} // namespace pw
#endif //POINTWORKS_UTILS_H

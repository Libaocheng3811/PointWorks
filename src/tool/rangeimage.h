//
// Created by LBC on 2025/1/9.
//

#ifndef POINTWORKS_RANGEIMAGE_H
#define POINTWORKS_RANGEIMAGE_H

#include "ui/base/customdialog.h"

#include "algorithm/keypoints.h"

#include <pcl/range_image/range_image.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>

QT_BEGIN_NAMESPACE
namespace Ui
{
    class RangeImage;
}
QT_END_NAMESPACE

class RangeImage : public ct::CustomDialog
{
    Q_OBJECT

public:
    explicit RangeImage(QWidget* parent = nullptr);

    ~RangeImage();

    virtual void init();
    virtual void reset();

    /**
     * @brief 报错小插曲：这里在定义ct::RangeImage::Ptr时一直报错error C2039: "RangeImage": 不是 "ct" 的成员，
     * 但是我正确引用了头文件并且对应的命名空间也书写正确，但就是会报错
     * ---原因：头文件保护的问题，因为我在不同的文件夹中创建了同名的文件keypoints.h,但是它们却有相同的头文件保护，导致modules/keypoints.h未能被正确引用
     * @return
     */
    ct::RangeImage::Ptr getRangeImage() {return m_range_image; }

public slots:
    void updateRangeImage(Eigen::Affine3f);

private:
    Ui::RangeImage* ui;
    ct::RangeImage::Ptr m_range_image;
    // vtkImageData 是 VTK（Visualization Toolkit）库中的一个类，用于表示图像数据
    vtkSmartPointer<vtkImageData> m_image_data;
    vtkSmartPointer<vtkRenderer> m_ren;
    // vtkImageActor 是 VTK 库中的一个类，用于表示图像的可视化对象（Actor）,用于在渲染窗口中显示图像
    vtkSmartPointer<vtkImageActor> m_actor;
    ct::Cloud::Ptr m_cloud;
};

#endif //POINTWORKS_RANGEIMAGE_H

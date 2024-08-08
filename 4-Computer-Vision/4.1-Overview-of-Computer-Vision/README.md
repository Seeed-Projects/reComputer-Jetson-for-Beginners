# Overview of Computer Vision

Computer Vision is a field of Artificial Intelligence (AI) that enables computers to "see." Just as humans use their eyes to perceive the world and then use their brains to understand what they see, computer vision allows computers to "see" images through cameras and then "understand" the content of those images through programming.

<p align="center">
    <img src="./images/computer-vision.png" height="400" alt="computer-vision">
</p>

Computer vision endows machines with human-like "visual capabilities," bringing significant transformation to many fields. It not only improves work efficiency but also helps solve complex problems, such as enhancing the accuracy of medical diagnoses, improving traffic safety, and playing a crucial role in the future of robotics.


## How Computer Vision Works

The working principles of computer vision are quite similar to the way the human brain works. It can be divided into three simple steps: `seeing`, `thinking`, and `acting`.

- **Seeing (Acquiring Images):** First, the computer needs to acquire images. Just like we use cameras to take photos, computers use cameras to capture pictures or videos. These images represent the world as seen through the computer's "eyes."

- **Thinking (Analyzing and Understanding):** Next, the computer needs to "think," meaning it needs to understand the content of the images. This step mainly relies on computer programs and algorithms. The program analyzes the pixels in the image and identifies shapes, colors, objects, and other information. For example, when seeing a photo of a dog, the program analyzes the features of the image and determines that it is a dog, not a cat or something else.

- **Acting (Taking Action):** Finally, the computer takes action based on the information it has understood. This action could be as simple as displaying the result, like telling you what is in the image; or it could be more complex, such as in self-driving cars, where detecting an obstacle ahead may trigger a warning or automatic braking.

With the rapid advancement of technology, artificial neural networks have become the preferred tools for analyzing and understanding images. More and more people are inclined to use this advanced technology because it can simulate the way the human brain works, automatically learning and extracting useful information from massive amounts of data. This makes machines more intelligent and efficient in handling visual tasks. 

## The History of Computer Vision Development

The history of computer vision can be divided into several key phases:

**Initial Stage (1960s-1980s):** Early research in computer vision focused on image processing and pattern recognition, primarily for simple image analysis tasks.

**Foundational Research Stage (1980s-1990s):** With advances in computer hardware and the emergence of machine learning methods, computer vision expanded to more complex tasks such as face recognition and object detection.

**Algorithm Innovation Stage (2000s):** The development of feature extraction methods and algorithms like Support Vector Machines (SVM) led to significant progress in specific areas, such as handwritten digit recognition.

**Deep Learning Stage (2010s-Present):** The application of deep learning, particularly Convolutional Neural Networks (CNNs), significantly improved performance in tasks like image classification, object detection, and image segmentation. This stage saw computer vision technologies being widely applied in areas like autonomous driving and medical imaging.

**Multimodal Fusion and Expanding Applications (2020s-Future):** Computer vision is increasingly integrating with other technologies, such as natural language processing, fostering new applications like visual question answering and multimodal analysis.

These stages highlight the evolution of computer vision from simple image processing tools to sophisticated visual understanding systems, continually driving advancements in the field of artificial intelligence.

## What are the tasks in computer vision?

<p align="center">
    <img src="./images/cv-tasks.gif" alt="computer-vision">
</p>

Computer vision encompasses a wide range of tasks, from basic image processing to complex scene understanding. Here are some of the main tasks in computer vision:

1. Image Classification: The task involves assigning an image to one or more predefined categories. For example, identifying whether an image contains a cat, dog, or other objects.
2. Object Detection: Not only identifies the categories of objects in an image but also determines their locations. The output typically includes bounding boxes and associated labels. For example, detecting and labeling all vehicles, pedestrians, etc., in an image.
3. Image Segmentation: This task involves dividing an image into different regions, usually to separate the foreground from the background. Semantic segmentation assigns each pixel to a category, while instance segmentation further distinguishes between different instances of the same category.
4. Face Recognition: Involves detecting and identifying faces in images or videos, commonly used for authentication and security monitoring.
5. Pose Estimation: Detects key points of a body or object to infer its pose. For example, human pose estimation can identify joint positions to infer a person's posture.
6. Optical Flow Estimation: Computes the motion vector of each pixel in a sequence of images, useful for analyzing dynamic scenes, such as video stabilization and object tracking.
7. 3D Reconstruction: Generates 3D models from 2D images, used in virtual reality, augmented reality, and architectural modeling.
8. Scene Understanding: Involves comprehensive understanding of the entire scene, including object relationships and scene classification. For example, determining whether an image is indoor or outdoor, urban or rural.
9. Visual Question Answering: A system answers questions based on the content of images, combining natural language processing and computer vision.
10. Image Generation: Uses generative models, such as Generative Adversarial Networks (GANs), to create new images, including tasks like image restoration and style transfer.
11. Image Super-Resolution: Generates high-resolution images from low-resolution inputs, aiming to restore fine details as much as possible.
12. Video Analysis: Involves tasks like object detection, tracking, and behavior recognition in videos, used in surveillance, entertainment, and sports analysis.

These tasks can often be combined, such as in autonomous vehicles, which require a combination of object detection, object tracking, and scene understanding to navigate complex environments.

## Application Scenarios of Computer Vision
Computer vision has numerous applications in our daily lives. Here are some common examples:

Face Recognition: On many smartphones, face recognition has become a way to unlock the device. Computer vision technology scans your face and compares it with stored facial data to confirm your identity.

Autonomous Driving: Autonomous vehicles use computer vision to "see" road conditions. Cameras capture information about the road, pedestrians, vehicles, and traffic signs, helping the car make driving decisions.

Medical Imaging: In hospitals, computer vision assists doctors in analyzing X-rays, CT scans, and other medical images. It helps in detecting diseases, such as identifying tumors or fractures.

Intelligent Surveillance: In surveillance systems, computer vision can recognize faces, detect unusual behavior, or identify dangerous situations, which helps enhance public safety.

Shopping Experience: In some modern stores, computer vision aids in cashless payments and self-checkout. After customers select items, the system automatically recognizes the products and processes the payment.

## Conclusion
Overall, computer vision is a technology that enables computers to understand and process visual information. Through this technology, we allow machines to "see" and comprehend the world, performing various tasks. This not only changes our way of life but also paves the way for future technological advancements.

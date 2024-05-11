from math import pi
import cv2
import numpy as np

class Symbols:
    def initTemplate(self, template_paths, template_names):
        # Load the template images
        self.template_paths = template_paths

        self.templates = []
        
        self.template_names = template_names

        for path in self.template_paths:
            template = cv2.imread(path, 0)
            if template is None:
                print(f"Failed to load template image from: {path}")
                continue
            self.templates.append(template)

        # Choose the method for comparison
        self.method = cv2.TM_CCOEFF_NORMED
        
        self.TemplateResult = 0
        
        self.toggle = 0
    
    def matchTemplate(self, tempimg):
        img = tempimg
        
        # Convert the frame to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Iterate over each template and perform template matching
        for idx, template in enumerate(self.templates):
            w, h = template.shape[::-1]
            res = cv2.matchTemplate(gray, template, self.method)
            min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
            
            # Calculate confidence based on the maximum value obtained from res
            confidence = max_val

            # Draw rectangle and put text if confidence is greater than a threshold
            if confidence > 0.85:  # Adjust threshold as needed
                top_left = max_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)
                #cv2.rectangle(img, top_left, bottom_right, (0, 255, 0), 2)
            
                # Put text indicating the matched object and confidence rate
                text = f"{self.template_names[idx]} - Confidence: {confidence:.2f}"
                cv2.putText(img, text, (top_left[0], top_left[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)
                
                self.TemplateResult = img
                self.toggle = 1
                return 1

        self.toggle = 0
        return 0    
        
    def symbolDetection(self, symimg):
        img = symimg
        hsvFrame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        # Define color ranges for red, green, and blue
        
        # HSV from both sides of the spectrum for red to make it more accurate 
        red_lower = np.array([0, 0, 40], np.uint8) 
        red_upper = np.array([25, 255, 255], np.uint8)
        redtoo_lower = np.array([140, 0, 40], np.uint8)
        redtoo_upper = np.array([179, 255, 255], np.uint8)
        redtoo_mask = cv2.inRange(hsvFrame, redtoo_lower, redtoo_upper)
        red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)
        red_mask = cv2.bitwise_or(red_mask, redtoo_mask)
        
        green_lower = np.array([40, 125, 75], np.uint8) 
        green_upper = np.array([90, 255, 255], np.uint8) 
        green_mask = cv2.inRange(hsvFrame, green_lower, green_upper) 
        
        blue_lower = np.array([100, 80, 2], np.uint8) 
        blue_upper = np.array([130, 255, 255], np.uint8) 
        blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper) 

        kernel = np.ones((5, 5), "uint8") 

        # Dilate masks for better detection
        red_mask = cv2.dilate(red_mask, kernel) 
        green_mask = cv2.dilate(green_mask, kernel) 
        blue_mask = cv2.dilate(blue_mask, kernel) 

        # Find contours for each color
        contours_red, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Process contours for each color (you can modify this part based on your requirements)
        for contours in [contours_red, contours_green, contours_blue]:
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 10000: 
                    
                     # Epsilon is multiplied by perimeter of contour in order to approximate contours of all sizes equally
                    epsilon = 0.01 * cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, epsilon, True)
                    
                    x, y, w, h = cv2.boundingRect(approx)
                    x_mid = int(x + w / 3)
                    y_mid = int(y + h / 1.5)

                    coords = (x_mid, y_mid)
                    colour = (0, 0, 0)
                    font = cv2.FONT_HERSHEY_DUPLEX

                    if len(approx) > 8:
                        circularity = (4 * pi * area) / (cv2.arcLength(contour, True) ** 2)
                        if circularity < 0.7:
                            cv2.putText(img, "Partial Circle", coords, font, 1, colour, 1)
                        else:
                            cv2.putText(img, "Circle", coords, font, 1, colour, 1)       
                    elif len(approx) == 3:
                        cv2.putText(img, "Triangle", coords, font, 1, colour, 1)
                    elif len(approx) == 4:
                        cv2.putText(img, "Rectangle", coords, font, 1, colour, 1)
                    elif len(approx) == 5:
                        cv2.putText(img, "Pentagon", coords, font, 1, colour, 1)
                    elif len(approx) == 6:
                        cv2.putText(img, "Hexagon", coords, font, 1, colour, 1)
                        
                    else:
                        continue
                  
                    cv2.drawContours(img, [contour], -1, (20, 193, 241), 3)
                else:
                    continue

        # Set the processed image as the result
        self.shapeResult = img



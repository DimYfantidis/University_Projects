import pygame
import numpy as np

from io import BytesIO
from PIL import Image


class ScreenInput:
    def __init__(self: 'ScreenInput', width: int, height: int, out_dim: tuple[int, int]) -> 'ScreenInput':
        # Screen width
        self.width = width
        # Screen Height
        self.height = height
        # Refresh rate
        self.fps = 300
        # Refresh rate clock
        self.timer = pygame.time.Clock()
        # List of white strokes performed on the black canvas
        self.painting = []
        # The screenshot of the canvas after finishing
        self.captured_image = None
        # Screenshot dimensions
        self.image_width, self.image_height = out_dim


    # Opens a pygame window 
    def display(self: 'ScreenInput') -> 'ScreenInput':
        pygame.init()

        img = pygame.image.load('./utils/brush_royalty_free.png')
        pygame.display.set_icon(img)
        pygame.display.set_caption('Draw a digit (exit to save)')

        # Open display window
        self.screen = pygame.display.set_mode([self.width, self.height])
        status = {'run': True,'brush_size': 14}

        while status['run']:
            self.timer.tick(self.fps)
            self.screen.fill('black')

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    status['run'] = False
            
            left_click = pygame.mouse.get_pressed()[0]
            right_click = pygame.mouse.get_pressed()[2]
            
            if left_click:
                mouse = pygame.mouse.get_pos()
                self.painting.append(mouse)
            if right_click:
                self.painting.clear()

            for i in range(len(self.painting)):
                pygame.draw.circle(self.screen, 'white', self.painting[i], status['brush_size'])
            
            pygame.display.flip()
        
        # After exiting, save a screenshot of the final frame in a buffer
        buffer = BytesIO()
        pygame.image.save(self.screen, buffer)

        # Transform the bytes in the buffer to an np.ndarray representation of a 28x28 grayscale image
        buffer.seek(0)
        self.captured_image = np.array(Image.open(buffer).resize((self.image_width, self.image_height)).convert('L'))

        # Release resources
        buffer.close()
        pygame.quit()
        return self
    
    
    # Fetch the screenshot of the last frame
    def get_drawn_image(self: 'ScreenInput') -> np.ndarray:
        return self.captured_image

import pygame
import sys

def draw_text_input_box(screen, font, input_box, text):
    pygame.draw.rect(screen, (255, 255, 255), input_box, 2)
    font_surface = font.render(text, True, (255, 255, 255))
    screen.blit(font_surface, (input_box.x + 5, input_box.y + 5))

def main():
    pygame.init()

    width, height = 800, 600
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption('Text Input Box')

    clock = pygame.time.Clock()

    font = pygame.font.Font(None, 36)
    input_box = pygame.Rect(50, 50, 200, 32)
    text = ''

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RETURN:
                    print(text)  # You can do something with the entered text here
                    text = ''
                elif event.key == pygame.K_BACKSPACE:
                    text = text[:-1]
                else:
                    text += event.unicode

        screen.fill((0, 0, 0))
        draw_text_input_box(screen, font, input_box, text)

        pygame.display.flip()
        clock.tick(30)

if __name__ == "__main__":
    main()

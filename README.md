# hsrb-display

### Meadia_display
Main component. Coordinates TextToImage and Image displays son the HSR via two topics
- publish text on the topic "head_display/text_to_image" to generate an Image with that text on HSR-Display
- publish predefined image id on topic "/media_switch_topic" to display the image on hsr screen

### subscriber
This file contains a subscriber that the media_display file publishes to to get images published on the 
HSR display. 

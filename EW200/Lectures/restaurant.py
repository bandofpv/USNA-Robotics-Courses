class Restaurant():
    def __init__(self, restaurant_name, cuisine_type):
        self.restaurant_name = str(restaurant_name).title()
        self.cuisine_type = str(cuisine_type).title()
        self.number_served = 0
        
    def describe_restaurant(self):
        print(f"{self.restaurant_name} makes {self.cuisine_type} food")
        
    def open_restaurant(self):
        print(f"{self.restaurant_name} is open!")
    
    def set_number_served(self, number_served):
        self.number_served = number_served
    
    def increment_number_served(self, increment):
        self.number_served += increment
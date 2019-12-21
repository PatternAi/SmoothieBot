import sys


class Recipe:
	""" Recipe class for EECS 106a SmoothieBot Project"""
	available_fruits = ['apple','orange','banana','grapes']
	fruit_counts = {'apple':5,'orange':5,'banana':4,'grapes':5}

	smoothie1 = {'apple':2,'orange':1,'banana':1}
	smoothie2 = {'apple':1,'orange':0,'banana:':2}
	smoothie3 = {'apple':2,'orange':2,'banana:':0}

	smoothies = {'smoothie1':smoothie1,
				 'smoothie2':smoothie2,
				 'smoothie3':smoothie3}

	def __init__(self):
		self.order = None
		self.build_order()
		self.dict_to_list()

	def build_order(self):
		order_type = input('Building smoothie recipe...\ncustom or premade: ')
		if order_type == 'custom':
			self.custom_order()
		elif order_type == 'premade':
			self.premade_order()
		else:
			print('Invalid smoothie request!')
			print('Smoothie type must be either "custom" or "premade"')
			print('Restarting order process')
			print('-'*30+'\n\n')
			self.build_order()

	def premade_order(self):
		self.display_smoothies()
		smoothie_type = input('Place order: ')
		if smoothie_type not in self.smoothies.keys():
			print('Invalid smoothie request! Smoothie "%s" does not exist!' % smoothie_type)
			print('Restarting order process')
			print('-'*30+'\n\n')
			self.build_order()
		else:
			self.order = self.smoothies[smoothie_type]

	def custom_order(self):
		order = {}
		print('Input desired # of fruit')
		for fruit in self.available_fruits:
			amount = input(fruit+': ')
			if amount == '':
				amount = 0
			amount = int(amount)
			if amount > self.fruit_counts[fruit]: 
				print('Invalid smoothie request! Too many %s!' % fruit)
				print('Max amount of {} is {}.'.format(fruit, self.fruit_counts[fruit]))
				print('Restarting order process')
				print('-'*30+'\n\n')
				self.build_order()
			else:
				order[fruit] = amount

		self.order = order

	def display_smoothies(self):
		print("Here are the available smoothies:")
		for smoothie, ingredients in self.smoothies.items():
			print(smoothie)
			print(ingredients)
			print()
	
	def get_order(self):
		"""Get smoothie order as python dictionary"""
		return self.order

	def dict_to_list(self):
		"""Change dict order to list order"""
		res = []
		for k,v in self.get_order().items():
			res.extend([k]*v)
		self.order = res

if __name__ == '__main__':
	R = Recipe()
	print(R.order)

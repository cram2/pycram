from .dependencies import *


class Location(Base):
    """
    A role classifying a location of interest, often specified as a spatial relation between several objects, themselves usually classified by spatial relation roles.
    """
    

class Room(Base):
    """
    Space that can be occupied or where something can be done.
    """
    

class Entity(Base):
    """
    Anything: real, possible, or imaginary, which some modeller wants to talk about for some purpose.
    """
    

class Robot(Base):
    ...
    

class Appliance(Base):
    """
    A device designed to perform a specific task, and that can be operated in some way.
    """
    

class BakedGood(Base):
    ...
    

class Blade(Base):
    """
    A flat cutting edge of an object used to cut through other objects, such as the blade of a ventilator cutting through the air.
    """
    

class Bottle(Base):
    """
    A container with a narrow neck used to store fluids.
    """
    

class Bowl(Base):
    """
    A round, deep object used to contain food or liquid.
    """
    

class RoboCupDishes(Base):
    ...
    

class Fragility(Base):
    ...
    

class Box(Base):
    """
    A cuboid-shaped container.
    """
    

class Bread(Base):
    ...
    

class BreakfastPlate(Base):
    """
    A smaller plate used for serving bread at the dining table with the main meal or to serve breakfast.
    """
    

class CerealBox(Base):
    """
    A box which holds, or at least is intended to usually hold, cereal flakes.
    """
    

class RoboCupSnacks(Base):
    ...
    

class Cup(Base):
    """
    A round shaped container used for storing liquid and drinking out of it, typically has  a handle for grasping.
    """
    

class Cupboard(Base):
    """
    A piece of furniture for storing things.
    """
    

class Cutlery(Base):
    ...
    

class HouseholdHardware(Base):
    """
    Household hardware (or simply, hardware) is equipment that can be touched or held by hand such as keys, locks, nuts, screws, washers, hinges, latches, handles, wire, chains, belts, plumbing supplies, electrical supplies, tools, utensils, cutlery and machine parts.
    """
    

class Kitchen(Base):
    """
    A designated room to perform the task of cooking.
    """
    

class CuttingTool(Base):
    ...
    

class Deposition(Base):
    """
    The disposition to support objects.
    """
    

class DesignedChair(Base):
    """
    A piece of furniture designed for one person to sit on.
    """
    

class DesignedComponent(Base):
    """
    An object designed to be part or element of a larger whole.
    """
    

class DesignedContainer(Base):
    """
    An item designed to be able to hold some other items, preventing their free movement and/or protecting them from outside influence. Containers may be used for storage, or to obtain control over items that are otherwise hard to manipulate directly (e.g. liquids).
    """
    

class DesignedFurniture(Base):
    """
    An object used to make a room or building suitable for living or working.
    """
    

class DesignedHandle(Base):
    """
    An item designed to fit well within a grasping hand, often attached to another item to enhance its manipulability.
    """
    

class DesignedTool(Base):
    """
    An item designed to enable some action, in which it will play an instrumental role.
    """
    

class DinnerPlate(Base):
    """
    A regular size plate for eating a main meals.
    """
    

class Dish(Base):
    ...
    

class Dishwasher(Base):
    """
    An appliance for washing dishes and cutlery automatically.
    """
    

class DishwasherTab(Base):
    """
    A solid detergent inserted into dishwashers.
    """
    

class RoboCupCleaningSupplies(Base):
    ...
    

class Disposition(Base):
    """
    The tendency of an object (the bearer) to make certain events happen with others (the triggers).
    extrinsic
    """
    

class Door(Base):
    """
    A hinged, sliding, or revolving barrier at the entrance to a building, room, or vehicle, or in the frame of a cupboard.
    """
    

class Drawer(Base):
    """
    A storage compartment without a lid, made to slide horizontally in and out of a piece of furniture.
    """
    

class Fluid(Base):
    """
    A substance with a consistency such that it can flow or diffuse.
    """
    

class Fork(Base):
    """
    Cutlery with two or more prongs used for lifting food to the mouth or holding it when cutting.
    """
    

class Glass(Base):
    """
    A container made usually from glass or transparent plastic which is used to hold, typically cold, beverages.
    """
    

class Intrinsic(Base):
    """
    A physical quality that is independent of context.
    intrinsic
    """
    

class Jar(Base):
    """
    A container used for long-term storage of some liquid or viscous dish.
    """
    

class KitchenCabinet(Base):
    """
    A cupboard designed to be used in kitchen environments.
    """
    

class Cabinet(Base):
    ...
    

class KitchenKnife(Base):
    """
    A tool designed to be used for kitchen related common tasks. Such as smearing a bread or cutting a cucumber.
    """
    

class Knife(Base):
    """
    An instrument composed of a blade fixed into a handle, used for cutting or as a weapon.
    """
    

class Lid(Base):
    """
    A removable or hinged cover for the top of a container.
    """
    

class MilkBottle(Base):
    """
    A bottle which holds, or at least is intended to usually hold, milk.
    """
    

class Milk(Base):
    """
    Milk is a nutrient-rich liquid that comes from cows or other mammals and is commonly consumed by humans.
    """
    

class RoboCupDrinks(Base):
    ...
    

class MilkPack(Base):
    """
    A pack which holds, or at least is intended to usually hold, milk.
    """
    

class Pack(Base):
    """
    A small cardboard or paper container.
    """
    

class Pan(Base):
    """
    A container used for cooking food.
    """
    

class Pancake(Base):
    ...
    

class PastaBowl(Base):
    """
    A bowl which holds, or at least is intended to usually hold, pasta.
    """
    

class Plate(Base):
    """
    A flat and usually circular object from which food is eaten or served.
    """
    

class Pot(Base):
    """
    A container used for cooking food.
    """
    

class Preference(Base):
    """
    A 'Preference' is a 'Quality' of an 'Agent' that orders 'Situation's by some heuristic based on the happiness, satisfaction, gratification, morality, enjoyment, and utility (see alse https://en.wikipedia.org/wiki/Preference) they provide to their bearing Agent.
    
    The pattern is as follows: A 'Preference' 'is described by' a 'Predilection', which also 'describes' an 'Order' that 'orders' 'Order item's that contain only 'Situation's. The 'Situation's then are modeled according to what the preference entails.
    
    That a 'Preference' orders 'Situation's might be unintuitive, but makes the model very general. A few examples:
    
    Example 1
    
    "Peter likes coffee and dislikes tea".
    Here, between different hypothetical situations where he plays the role of a performer in a drinking task, Peter prefers the situations in which role of the drunken object is played by some coffee (vs. some tea). Note that the coffe and tea are hypothetical objects as well and could, for example, be represented via reified Concepts.
    
    Example 2
    
    "Would you like this pot of coffee, or this pot of tea, Peter?"
    Here, as opposed to Example 1, the pot of coffee and the pot of tea are not hypothetical, but concrete.
    
    Example 3
    
    "Would you like this pot of coffee, or should I brew you some tea?"
    Here, the pot of coffee is concrete and the tea is not.
    
    Example 4
    
    Situations are not restricted to Tasks; other event types are possible as well.
    For example, Peter might prefer the Containment State of a tiger being inside a cage vs. the Containment State of the tiger being outside of the cage.
    """
    

class Rack(Base):
    """
    A frame for holding or storing things.
    """
    

class Refrigerator(Base):
    """
    An appliance which is artificially kept cool and used to store food and drink.
    """
    

class SaladBowl(Base):
    """
    A bowl which holds, or at least is intended to usually hold, salad.
    """
    

class SaltShaker(Base):
    """
    A shaker which holds, or at least is intended to usually hold, salt.
    """
    

class Shaker(Base):
    """
    A container used for storing fine grained substances such as salt or paper.
    """
    

class Sink(Base):
    """
    A large bowl used to direct the flow of liquid into a drain, typically has a tap used to fill the sink with water, and a plug used to block the drain.
    """
    

class Sofa(Base):
    """
    A comfortable seat for two or more people to sit on.
    """
    

class Spatula(Base):
    ...
    

class Spoon(Base):
    """
    An eating or cooking implement consisting of a small shallow bowl with a relatively long handle.
    """
    

class Table(Base):
    """
    A piece of furniture with a flat top and one or more legs.
    """
    

class TrashContainer(Base):
    """
    An item designed to hold trash. Typically equipped with a mechanism to close or open it, to prevent smells from the trash from propagating out and insects and other pests to get in.
    """
    

class WaterGlass(Base):
    """
    A glass which holds, or at least is intended to usually hold, water. Also affords drinking from it.
    """
    

class WineBottle(Base):
    """
    A bottle which holds, or at least is intended to usually hold, wine.
    """
    

class WineGlass(Base):
    """
    A glass which holds, or at least is intended to usually hold, wine. Also affords drinking from it.
    """
    

class AbrasiveSponge(Base):
    """
    An abrasive sponge is a cleaning tool designed to scrub away tough dirt and grime from surfaces using its rough, abrasive surface. It is typically made from materials such as nylon, polyester, or natural fibers, and is commonly used in the kitchen and bathroom for cleaning dishes, pots, pans, and other surfaces.
    """
    

class Sponge(Base):
    """
    A sponge is a porous material that is typically used for cleaning surfaces or absorbing liquid.
    """
    

class AdjustableWrench(Base):
    """
    An adjustable wrench is a hand-held tool with a movable jaw that can be adjusted to fit nuts, bolts, and other objects of different sizes, used for turning or tightening them.
    """
    

class Wrench(Base):
    """
    A wrench is a hand tool with a handle and adjustable jaws or a fixed socket used for gripping and turning nuts, bolts, and other fasteners.
    """
    

class Apple(Base):
    """
    An apple is a round or oval-shaped fruit with a red, green, or yellow skin, a crisp, juicy flesh, and a core with seeds in the center.
    """
    

class Fruit(Base):
    """
    Fruit is the sweet and fleshy product of a tree or other plant that contains seed and can be eaten as food.
    """
    

class RoboCupFruits(Base):
    ...
    

class AppleJuice(Base):
    """
    Apple juice is a beverage made by pressing or extracting the natural liquid from apples, a sweet and crunchy fruit. It is often consumed as a refreshing drink.
    """
    

class Juice(Base):
    """
    Juice is a beverage made from the extraction or pressing of the natural liquid contained in fruits and vegetables. It can be consumed as a refreshing drink, and is often marketed as a source of vitamins and other nutrients.
    """
    

class Arena(Base):
    """
    The whole arena where the robot is normally allowed to move.
    """
    

class Bag(Base):
    """
    A bag is a container made of flexible or rigid materials, such as cloth, paper, or plastic, used to store or transport objects. It often has an opening at the top and can be closed with a zipper, button, or other fastening mechanism. Bags come in a variety of shapes and sizes and are used for a wide range of purposes, from carrying groceries and books to holding personal belongings when traveling.
    """
    

class Balcony(Base):
    ...
    

class Ball(Base):
    """
    A ball is a spherical object typically used in sports, games, or as a toy, made from various materials and usually bounced, thrown, or kicked.
    """
    

class Banana(Base):
    """
    A banana is a long, curved fruit with a yellow skin that is easy to peel, a soft, sweet flesh, and small seeds inside.
    """
    

class Baseball(Base):
    """
    A baseball is a ball used in the sport of baseball, made of cork or rubber, and covered with leather or synthetic materials, and is roughly the size of an adult's fist.
    """
    

class RoboCupToys(Base):
    ...
    

class Bathroom(Base):
    ...
    

class Bed(Base):
    """
    A bed is a piece of furniture for sleeping or resting on that typically includes a mattress and a frame.
    """
    

class Bedroom(Base):
    """
    A bedroom is a private space within a dwelling primarily used for sleeping and relaxation. It typically contains a bed, storage for personal belongings, and may include additional furniture such as a dresser or nightstand.
    """
    

class Beer(Base):
    ...
    

class Drink(Base):
    """
    A drink is a liquid that is consumed for the purpose of quenching thirst, providing hydration, or for enjoyment. Drinks can be alcoholic or non-alcoholic and can be served hot or cold, and come in a wide variety of flavors and types.
    """
    

class Bib(Base):
    ...
    

class DesignedFabric(Base):
    """
    Fabric is a flexible, woven material consisting of a network of natural or artificial fibers, often used for making clothing and other textile products.
    """
    

class BlankCreditCard(Base):
    """
    A blank credit card is a standard-size rectangular piece of plastic with no personal or financial information embossed or encoded on it, used as a blank template for creating customized credit, debit, or ID cards.
    """
    

class CreditCard(Base):
    """
    A credit card is a standard-size rectangular piece of plastic. It is a payment card that allows the cardholder to borrow funds from a financial institution to make purchases and pay them back with interest over time.
    """
    

class BleachCleanserBottle(Base):
    """
    A bleach cleanser bottle is a container designed to hold a cleaning solution that contains bleach, a powerful disinfectant and stain remover. It typically has a nozzle or cap that dispenses the solution in a controlled manner to effectively clean and sanitize surfaces.
    """
    

class CleanserBottle(Base):
    """
    A container typically made of plastic or glass that holds cleaning products such as liquid soap, bleach, or other household chemicals.
    """
    

class Block(Base):
    """
    A block is a solid piece of hard material, such as wood, stone or metal, that is used as a construction material, as a support or to hold other objects.
    """
    

class Blouse(Base):
    ...
    

class Clothing(Base):
    ...
    

class BlueJeans(Base):
    ...
    

class Bolt(Base):
    """
    A bolt is a cylindrical or cone-shaped metal fastener with a threaded end used for securing two objects together, often requiring a nut and washer to hold it in place.
    """
    

class Fastener(Base):
    """
    A fastener is a hardware device used to join two or more objects together, allowing for the creation of non-permanent joints that can be easily removed or disassembled.
    """
    

class MetalHardware(Base):
    """
    Metal hardware refers to various metal components, such as nuts, bolts, screws, washers, and hinges, used in manufacturing and construction to join or secure different objects or materials together.
    """
    

class Book(Base):
    """
    A book is a written or printed work consisting of pages bound together, typically containing a story, information, or other forms of expression.
    """
    

class PrintedMatter(Base):
    """
    Printed matter refers to any material that has been printed, including books, magazines, newspapers, brochures, flyers, posters, and other forms of printed media.
    """
    

class Bookcase(Base):
    """
    A bookcase is a piece of furniture with horizontal shelves used to store books.
    """
    

class Shelf(Base):
    """
    A shelf unit is a piece of furniture consisting of several shelves, designed to hold books, decor, and other items and is typically made of wood or metal.
    """
    

class BoxLid(Base):
    """
    A box lid is a removable cover that fits over the top of a box, providing protection and keeping the contents secure.
    """
    

class Brand(Base):
    """
    A brand is a is a name, term, design, symbol or any other feature that identifies and distinguishes a company, product or service from other companies or products in the same market. It is the unique identity that represents the company and its reputation, and can influence consumer behavior and loyalty.
    """
    

class SocialObject(Base):
    """
    Any Object that exists only within some communication Event, in which at least one PhysicalObject participates in. 
    In other words, all objects that have been or are created in the process of social communication: for the sake of communication (InformationObject), for incorporating new individuals (SocialAgent, Place), for contextualizing or intepreting existing entities (Description, Concept), or for collecting existing entities (Collection).
    Being dependent on communication, all social objects need to be expressed by some information object (information objects are self-expressing).
    """
    

class Can(Base):
    """
    A can is a cylindrical container usually made of metal or aluminum, used for storing food, beverages, or other products.
    """

class Fridge(Base):
    ...

class Candle(Base):
    ...
    

class RoboCupDecorations(Base):
    ...
    

class DesignedSubstance(Base):
    ...
    

class Candy(Base):
    ...
    

class Sweets(Base):
    """
    Sweets, also known as candy, are a type of confectionery made from sugar or a sugar substitute, and often flavored with fruit, chocolate, or other ingredients. They come in a wide variety of forms such as hard candies, gummies, caramels, and chocolates, and are enjoyed as a treat or dessert.
    """
    

class Cereal(Base):
    """
    Cereal is a type of food made from processed grains that is typically eaten for breakfast.
    """
    

class ProcessedFood(Base):
    """
    Processed food refers to food that has been altered in some way from its natural state through various methods such as canning, freezing, cooking, or adding preservatives and additives. It often contains high amounts of salt, sugar, and fat, and is typically less nutritious than fresh, unprocessed food.
    """
    

class CerealBowl(Base):
    """
    A cereal bowl is a deep dish used for serving breakfast cereal with milk.
    """
    

class CerealBoxRoboCup(Base):
    """
    The cereal box(es) used at RoboCup 2023 Bordeaux
    """
    

class Chain(Base):
    """
    A chain is a series of metal links or rings, typically made of steel or other metals, connected together to form a flexible, strong, and durable tool for lifting, towing, securing, or transmitting power.
    """
    

class Chair(Base):
    ...
    

class CheezIt(Base):
    ...
    

class CheezItCrackerBox(Base):
    """
    A box containing, or at least intended to contain, Cheeze-It brand crackers. A cracker is a flat, dry-baked pastry, usually made from flour.
    """
    

class CrackerBox(Base):
    """
    A box containing, or at least intended to contain, crackers. A cracker is a flat, dry-baked pastry, usually made from flour.
    """
    

class Chili(Base):
    ...
    

class ChiliConCarne(Base):
    ...
    

class ChiliSinCarne(Base):
    ...
    

class ChipsCan(Base):
    """
    Chips can refers to a cylindrical container that holds potato chips.
    """
    

class ChocolateJello(Base):
    ...
    

class Jello(Base):
    """
    Jello, also known as gelatin, is a sweet dessert made by combining gelatin powder with hot water, sugar, and flavorings, then cooling and setting the mixture in a mold. It is a popular treat for both children and adults, and comes in a variety of flavors and colors.
    """
    

class Clamp(Base):
    """
    A clamp is a mechanical device with two jaws that can be adjusted to hold an object in place, used for woodworking, metalworking, and other types of manual work.
    """
    

class ForceConcentratingTool(Base):
    """
    Force-concentrating tools are hand tools designed to concentrate a force onto a small area to enable the user to exert a greater amount of force than would otherwise be possible.
    """
    

class CleaningProduct(Base):
    ...
    

class CleaningTool(Base):
    """
    A cleaning tool is an instrument used for cleaning or scrubbing surfaces.
    """
    

class Cleanser(Base):
    ...
    

class ClearBox(Base):
    """
    A clear box is a transparent container made of plastic, glass, or other clear materials, designed to hold and display objects while keeping them visible and protected.
    """
    

class CloseAction(Base):
    ...
    

class ClosedState(Base):
    ...
    

class Coat(Base):
    ...
    

class CoatHanger(Base):
    ...
    

class Rack(Base):
    """
    A rack is a framework or support structure used for holding or organizing various objects or equipment in a specific arrangement or pattern.
    """
    

class CoatRack(Base):
    """
    A coat rack is a free-standing or wall-mounted piece of furniture that is designed to hold coats, hats, scarves, and other outerwear.
    """
    

class CocoaBox(Base):
    """
    A cocoa box is a container, usually made of cardboard, metal, or plastic, designed to store and dispense cocoa powder. It often features a resealable lid or cap to maintain the freshness of the cocoa and prevent moisture from entering, and is valued for its ease of storage and use in household kitchens.
    """
    

class Coffee(Base):
    """
    Coffee is a beverage made by brewing roasted coffee beans, which come from the fruit of the Coffea plant. It is a popular drink worldwide, known for its stimulating effects due to its caffeine content, and can be served hot or cold in a variety of preparations.
    """
    

class CoffeeCan(Base):
    """
    A coffee can is a cylindrical container typically made of metal or plastic, used for storing and transporting coffee or other dry goods.
    """
    

class RoboCupFood(Base):
    ...
    

class CoffeeGrounds(Base):
    ...
    

class CoffeePack(Base):
    """
    A coffee pack typically refers to a package containing ground coffee or coffee beans used for making coffee, usually in a drip coffee maker, espresso machine, or French press. The packaging is designed to keep the coffee fresh and flavorful, and often includes information on the type of roast, flavor profile, and origin of the coffee.
    """
    

class CoffeeTable(Base):
    ...
    

class Cola(Base):
    ...
    

class ColaBottle(Base):
    ...
    

class ColaCan(Base):
    """
    A cola can is a small, cylindrical container, often made of aluminum, used for holding and distributing carbonated soft drinks, particularly cola. It features a pull-tab or stay-on-tab at the top for easy opening and consumption, and is popular for its portability and recyclability.
    """
    

class ColoredWoodBlock(Base):
    """
    Colored wood blocks are toys that consist of various sized wooden blocks that are painted in different colors and shapes for children to play with.
    """
    

class WoodBlock(Base):
    """
    A wood block is a solid piece of wood cut into a rectangular or square shape, used for carving, chopping, or as a decorative item.
    """
    

class Condiment(Base):
    """
    A condiment is a food item, typically a sauce or seasoning, that is used to enhance the flavor of other foods. Examples of condiments include ketchup, mustard, mayonnaise, soy sauce, and hot sauce.
    """
    

class Cookie(Base):
    """
    Cookies are baked sweet treats that are typically small, flat, and round in shape, and can be crispy, chewy, or soft. They are made from a dough that typically contains flour, sugar, eggs, and butter or oil, and can be flavored with ingredients such as chocolate chips, nuts, or spices.
    """
    

class CookieBox(Base):
    """
    A cookie box is a container made of cardboard or other materials that is used to store and transport cookies. It usually has a lid that can be opened and closed for easy access to the cookies inside.
    """
    

class CookieMix(Base):
    """
    A cookie mix is a pre-made dry mix that typically contains flour, sugar, baking powder or soda, and sometimes additional flavorings or ingredients such as chocolate chips, nuts, or dried fruit. It is used to make cookie dough quickly and easily by adding liquid ingredients such as eggs, butter, or oil, and can be baked into a variety of cookie shapes and flavors.
    """
    

class Cornflakes(Base):
    ...
    

class CornyBox(Base):
    """
    A box that contains cereal bars named "corny"
    """
    

class Couch(Base):
    ...
    

class CouchTable(Base):
    ...
    

class CrispsBag(Base):
    ...
    

class CupBlue(Base):
    ...
    

class CupGreen(Base):
    ...
    

class CupSmall(Base):
    ...
    

class Curry(Base):
    ...
    

class Customer(Base):
    """
    A customer is a person, whose name we know.
    """
    

class NaturalPerson(Base):
    """
    A person in the physical commonsense intuition: 'have you seen that person walking down the street?'
    """
    

class CustomerID(Base):
    """
    An id for every customer we meet.
    """
    

class CuttingBoard(Base):
    ...
    

class DesignedArtifact(Base):
    """
    A PhysicalArtifact that is also described by a Design. This excludes simple recycling or refunctionalization of natural objects. Most common sense 'artifacts' can be included in this class: cars, lamps, houses, chips, etc.
    """
    

class Desk(Base):
    ...
    

class DetectAction(Base):
    ...
    

class Dice(Base):
    """
    Dice are small, cube-shaped objects with markings on each face, typically used in games of chance and gambling, to generate random numbers, or for other recreational purposes.
    """
    

class Toy(Base):
    """
    A toy is an object designed for children or adults to play with, often for amusement or education, and can range from simple objects to complex electronic devices.
    """
    

class DiningRoom(Base):
    ...
    

class DiningTable(Base):
    """
    A table mainly used for dining. Often placed inside of a living room.
    """
    

class DinnerTable(Base):
    ...
    

class DishwasherTray(Base):
    """
    A dishwasher tray is a removable component in a dishwasher that holds dishes, glasses, and utensils during the washing cycle. It is designed with compartments and prongs to securely position and separate items for efficient cleaning.
    """
    

class DominoSugarBox(Base):
    """
    A box containing, or at least usually intended to contain, Domino brand sugar.
    """
    

class SugarBox(Base):
    """
    A box containing, or at least usually intended to contain, sugar.
    """
    

class Dress(Base):
    ...
    

class Liquid(Base):
    """
    Liquid food is a type of food that is in a liquid state and can be consumed without chewing.
    """
    

class DubbelFris(Base):
    ...
    

class Fanta(Base):
    ...
    

class FantaCan(Base):
    ...
    

class Fish(Base):
    """
    Fish are aquatic animals that are found in oceans, rivers, lakes, and other bodies of water. They come in a variety of shapes, sizes, and colors and are an important food source for humans and many other animals.
    """
    

class Food(Base):
    """
    Food refers to any substance consumed by living organisms that provide nutritional support for the body's growth, development, and maintenance.
    """
    

class FlatScrewdriver(Base):
    """
    A flat screwdriver is a hand tool characterized by a flat and narrow blade with a tip that fits into slotted screw heads. It is used for turning or tightening screws that have a single straight slot, commonly found in various mechanical and household applications.
    """
    

class ScrewDriver(Base):
    """
    A screwdriver is a hand tool consisting of a handle and a shaft with a shaped tip designed to fit into the corresponding slot or recess of a screw head. It is used for turning screws to fasten or loosen them in a wide range of applications, from construction and carpentry to electronics and household repairs.
    """
    

class Flavor(Base):
    """
    Flavor refers to the sensory experience of food or drink, specifically its taste and aroma.
    """
    

class FoamBrick(Base):
    """
    A foam brick is a soft, rectangular block made of foam, used for support or cushioning, often in sports or fitness activities.
    """
    

class BiologicalObject(Base):
    ...
    

class Footlocker(Base):
    """
    A type of storage trunk used to hold clothing and other personal items.
    """
    

class FrenchsYellowMustardBottle(Base):
    """
    French's Mustard bottle refers to a squeezable container that dispenses French's brand of yellow mustard.
    """
    

class MustardBottle(Base):
    """
    Mustard bottle refers to a squeezable container that dispenses mustard.
    """
    

class GelatineBox(Base):
    ...
    

class GinTonic(Base):
    ...
    

class GingerAle(Base):
    ...
    

class GlassCleanerSprayBottle(Base):
    """
    A glass cleaner spray bottle is a container designed to hold a cleaning solution specifically formulated for cleaning glass surfaces. It typically has a spray nozzle that dispenses the solution in a fine mist for easy application onto the surface.
    """
    

class SprayBottle(Base):
    """
    A spray bottle is a type of container that holds liquid and has a nozzle that can spray the liquid in a fine mist or stream.
    """
    

class Gloves(Base):
    ...
    

class GolfBall(Base):
    """
    A golf ball is a small, round ball used in the sport of golf, typically made of rubber and synthetic materials, with dimples on its surface to increase lift and reduce drag.
    """
    

class Grapes(Base):
    """
    Grapes are small round fruits that can be green or dark purple and can have seeds inside them.
    """
    

class GroceryBag(Base):
    ...
    

class Hagelslag(Base):
    ...
    

class Hallway(Base):
    ...
    

class HallwayCabinet(Base):
    ...
    

class Hammer(Base):
    """
    A hammer is a hand-held tool with a heavy metal head and a handle used for driving nails, breaking objects apart, or shaping metal.
    """
    

class HansanoWeidemilchMilkPack(Base):
    ...
    

class Hat(Base):
    ...
    

class Hoodie(Base):
    ...
    

class HotChocolate(Base):
    ...
    

class HygieneProduct(Base):
    """
    Hygiene products are items that are used to promote cleanliness and personal hygiene. This can include products such as soap, shampoo, toothpaste, deodorant, and feminine hygiene products.
    """
    

class IceTea(Base):
    """
    Iced tea is a beverage made by steeping tea leaves in hot water, then chilling the tea and serving it over ice. It can be sweetened and flavored with lemon, peach, or other fruits, and is a popular refreshment, especially during hot weather.
    """
    

class Tea(Base):
    """
    Tea is a hot or cold beverage made by steeping dried leaves from the tea plant, Camellia sinensis, in water. It is one of the most widely consumed beverages in the world and is enjoyed for its unique flavor, aroma, and potential health benefits.
    """
    

class IceTeaBottle(Base):
    """
    An iced tea bottle is a container, commonly made of plastic or glass, designed specifically for storing and dispensing iced tea. It typically features a screw-top or snap cap for resealability, allowing the beverage to stay fresh and chilled, and is favored for its portability and convenience.
    """
    

class IceTeaBox(Base):
    """
    An ice tea box refers to the container or packaging used to store and sell iced tea, usually in a ready-to-drink form. The box may contain individual servings or larger quantities, and can be made of various materials such as cardboard, plastic, or glass.
    """
    

class IceTeaCan(Base):
    ...
    

class IcedCoffee(Base):
    ...
    

class JaFettarmeHMilchMilkPack(Base):
    ...
    

class Jacket(Base):
    ...
    

class JellOChocolatePuddingBox(Base):
    """
    A box containing, or at least normally intended to contain, Jell-O brand chocolate flavored pudding powder.
    """
    

class PuddingBox(Base):
    """
    A box containing, or at least normally intended to contain pudding powder.
    """
    

class JellOStrawberryBox(Base):
    """
    A box containing, or at least normally intended to contain, Jell-O brand strawberry flavored gelatin.
    """
    

class JelloBox(Base):
    """
    A box containing, or at least normally intended to contain, jello.
    """
    

class Jug(Base):
    ...
    

class JuiceBottle(Base):
    """
    A juice bottle is a container, typically made from plastic or glass, that's specifically designed for storing and dispensing juice. It often features a screw-top or snap cap to maintain freshness and ease of use, and its portability makes it a popular choice for consumption at home or on the go.
    """
    

class JuiceBox(Base):
    """
    A juice box is a container made of paperboard, with a small attached plastic straw, used for packaging and drinking juice. They are typically single-serving and convenient for on-the-go consumption.
    """
    

class JuicePack(Base):
    """
    A juice pack is a portable container, often made of plastic or a combination of plastic, paper, and aluminum, designed to hold and dispense single servings of juice. It typically comes with a resealable cap or a pre-punched hole for a straw, providing a convenient option for on-the-go consumption and packed lunches.
    """
    

class KelloggsCornFlakes(Base):
    """
    Kellogg's Corn Flakes are a popular brand of breakfast cereal made from milled corn and malt flavoring. They were first created in the late 19th century by John Harvey Kellogg and his brother Will Keith Kellogg, and are known for their crunchy texture and mild, slightly sweet flavor.
    """
    

class KelloggsCornFlakesBox(Base):
    """
    A Kellogg's Corn Flakes box is a cardboard packaging that contains corn flakes, a popular breakfast cereal made from toasted corn.
    """
    

class Ketchup(Base):
    """
    Ketchup is a sweet and tangy condiment made from tomatoes, vinegar, sugar, and spices. It is commonly used as a topping for burgers, fries, and hot dogs, and is a popular ingredient in many sauces and dressings.
    """
    

class KitchenCounter(Base):
    ...
    

class KitchenTable(Base):
    """
    A table that is placed inside a kitchen and that is used for doing kitchen tasks like preparing meals.
    """
    

class Kiwi(Base):
    """
    Kiwi is a small, oval-shaped fruit with brown fuzzy skin and bright green flesh inside, packed with nutrients such as vitamin C, vitamin K, and potassium. It has a tangy, sweet flavor and is often eaten raw, added to salads or smoothies, or used as a garnish.
    """
    

class KoellnKnusperSchokoKeksBox(Base):
    """
    The Kölln Knusper Schoko & Keks box contains crunchy chocolate and cookie flakes that can be enjoyed as a breakfast cereal or snack.
    """
    

class Lamp(Base):
    ...
    

class LargeMarker(Base):
    """
    A large marker is a pen-like writing tool with a wide, felt tip used for creating bold, colorful marks or lettering on posters, signs, or other large surfaces.
    """
    

class WritingTool(Base):
    """
    A writing tool is a device used to inscribe characters onto a surface, such as paper or digital media, and can include pens, pencils, markers, styluses, and digital pens.
    """
    

class Leggings(Base):
    ...
    

class Lego(Base):
    """
    LEGO is a popular construction toy consisting of colorful interlocking plastic bricks and other components that can be assembled and reconfigured into various shapes and structures.
    """
    

class LegoDuplo(Base):
    """
    Lego Duplo is a line of Lego building toys designed for children aged 1.5 to 5 years old.
    """
    

class Lemon(Base):
    """
    A lemon is a yellow citrus fruit with a sour taste, acidic juice, and versatile use in cooking and beverages.
    """
    

class Lemonade(Base):
    ...
    

class Perishable(Base):
    ...
    

class Liquorice(Base):
    ...
    

class LivingRoom(Base):
    """
    A living room is a common area in a home designed for socializing, entertainment, and relaxation. It usually features comfortable seating, such as sofas and chairs, and may include a television, coffee table, and other furniture for gathering and conversation.
    """
    

class Lobby(Base):
    ...
    

class LookAtAction(Base):
    ...
    

class LoungeChair(Base):
    ...
    

class LowFatLongLastingMilk(Base):
    """
    Low-fat long-lasting milk is a type of milk that has been processed to remove some of the fat and extend its shelf life. This milk is typically heated to a high temperature, which kills off bacteria and enzymes that can cause spoilage, and then packaged in airtight containers.
    """
    

class Magazine(Base):
    """
    A magazine is a publication containing articles and illustrations, often on a particular subject or aimed at a particular audience.
    """
    

class Marble(Base):
    """
    A marble is a small, spherical toy made of glass, stone, or other materials, used in games, competitions, or for decoration.
    """
    

class MasterchefCan(Base):
    ...
    

class Mayonaise(Base):
    ...
    

class Meat(Base):
    ...
    

class MetalBowl(Base):
    """
    A metal bowl is a container made from metal material, typically stainless steel or aluminum, and used for various purposes such as mixing ingredients, serving food, or as a decorative object. It is durable, easy to clean, and can withstand high temperatures, making it ideal for use in the kitchen.
    """
    

class MetalCup(Base):
    ...
    

class MetalKnife(Base):
    ...
    

class MetalMug(Base):
    """
    A metal mug is a drinking vessel made from metal material, typically stainless steel or copper, and designed to hold hot or cold beverages. It is sturdy, reusable, and often used for camping, outdoor activities, or as a travel mug.
    """
    

class Mug(Base):
    """
    A mug is a cylindrical-shaped cup, usually made of ceramic, porcelain, or glass, with a handle for holding and drinking hot beverages such as coffee or tea.
    """
    

class MetalPlate(Base):
    ...
    

class Microwave(Base):
    """
    A microwave is an appliance that cooks or heats up food by using microwave radiation to excite the molecules in the food.
    """
    

class MilkCoffee(Base):
    ...
    

class MilkGlass(Base):
    """
    A glass which holds, or at least is intended to usually hold, water. Also affords drinking from it.
    """
    

class MilkJug(Base):
    ...
    

class MilkPackJa(Base):
    ...
    

class Mineral(Base):
    """
    A mineral is a naturally occurring inorganic substance that has a specific chemical composition and a crystalline structure. They are found in rocks and the Earth's crust and have many practical uses, such as being used as building materials, in electronics, and in the production of jewelry.
    """
    

class Substance(Base):
    """
    Any PhysicalBody that has not necessarily specified (designed) boundaries, e.g. a pile of trash, some sand, etc. 
    In this sense, an artistic object made of trash or a dose of medicine in the form of a pill would be a FunctionalSubstance, and a DesignedArtifact, since its boundaries are specified by a Design; aleatoric objects that are outcomes of an artistic process might be still considered DesignedArtifact(s), and Substance(s).
    """
    

class MineralWater(Base):
    ...
    

class MiniSoccerBall(Base):
    """
    A mini soccer ball is a smaller version of a regulation soccer ball, often used for training, indoor or recreational play, or for promotional or decorative purposes.
    """
    

class MoveTorsoAction(Base):
    ...
    

class Muesli(Base):
    ...
    

class MuesliBox(Base):
    ...
    

class MulledWineTea(Base):
    """
    Mulled wine tea is a type of tea that is typically made with a blend of black tea, spices, and dried fruit, such as orange peel or cinnamon. It is often served as a warm, comforting beverage during the winter months and is known for its rich, aromatic flavor and spicy notes.
    """
    

class Mustard(Base):
    """
    Mustard is a condiment made from the seeds of the mustard plant, which are ground into a powder and mixed with vinegar or other liquids to create a paste or sauce. It is commonly used as a flavoring for sandwiches, dressings, marinades, and many other dishes.
    """
    

class Nail(Base):
    """
    A nail is a thin, pointed metal or wood fastener with a flat head used for attaching or joining two objects together, or for securing an object to a surface.
    """
    

class Napkin(Base):
    """
    A square piece of cloth or paper used at a meal to wipe the fingers or lips and to protect garments
    """
    

class NavigateAction(Base):
    ...
    

class NesquickCerealBox(Base):
    ...
    

class NinePegHoleTest(Base):
    """
    A pegboard with nine holes used for cognitive and psychomotor testing.
    """
    

class Nut(Base):
    """
    A nut is a small, usually hexagonal-shaped metal or plastic fastener with a threaded hole used for securing a bolt or screw in place.
    """
    

class Office(Base):
    ...
    

class OpenAction(Base):
    ...
    

class OpenedState(Base):
    ...
    

class Orange(Base):
    """
    An orange is a round citrus fruit with a tough, bright orange skin, pulpy flesh with sections, and a juicy, tangy flavor.
    """
    

class OrangeJuice(Base):
    ...
    

class OrangeJuiceBox(Base):
    """
    An OrangeJuiceBox is a portable, typically single-serving container made of paperboard or other lightweight materials, designed to hold and dispense orange juice. It is commonly equipped with a straw or a pull-tab for easy consumption, and is popular for its convenience and use in packed lunches or on-the-go situations.
    """
    

class Oregano(Base):
    """
    Oregano is a fragrant herb commonly used in Mediterranean and Mexican cuisine. It has a strong, slightly bitter flavor and is often used dried to add depth to tomato-based sauces, meats, and vegetables.
    """
    

class Spice(Base):
    """
    Spices are a group of plant-based ingredients that are used to add flavor and aroma to food. They are typically dried and ground into powder or used fresh, and can be used alone or in combination with other spices to create complex flavors in dishes.
    """
    

class OreganoShaker(Base):
    """
    An oregano shaker is a container used for storing and dispensing dried oregano spice, which is commonly used in cooking to add flavor and aroma to dishes. The shaker typically has small holes or slots on the top to allow for controlled dispensing of the oregano flakes.
    """
    

class Package(Base):
    """
    A package is a wrapped or boxed object that contains one or more items for transport or storage. Packages can range in size from small envelopes to large boxes or crates.
    """
    

class PadlockAndKeys(Base):
    """
    A padlock and keys are a security device used for locking and securing doors, gates, or other objects, consisting of a metal lock with a U-shaped bar and a set of keys to open and close it.
    """
    

class PancakeMix(Base):
    ...
    

class PancakeMixBottle(Base):
    """
    A pancake mix bottle is a container that holds pre-made pancake batter, ready to be cooked. These bottles usually come with a nozzle for easy pouring and can be found in different sizes and shapes.
    """
    

class Pantry(Base):
    ...
    

class Pants(Base):
    ...
    

class PaperBag(Base):
    """
    A paper bag is a type of bag made from paper materials, such as kraft paper or recycled paper. It is commonly used for carrying groceries, snacks, and other lightweight items, and is often considered a more environmentally-friendly option compared to plastic bags. Paper bags can be recycled and are biodegradable, making them a sustainable choice for packaging and transporting goods.
    """
    

class ParkArmsAction(Base):
    ...
    

class PastryBox(Base):
    """
    A pastry box is a container used to transport and store pastries such as cakes, pies, and tarts. It is typically made of cardboard or plastic and designed to keep the pastries fresh and protected during transport, with features such as secure lids and inserts to prevent movement and damage.
    """
    

class PeaSoupCan(Base):
    ...
    

class SoupCan(Base):
    """
    Soup can refers to a cylindrical metal container that holds condensed or ready-to-eat soup.
    """
    

class Peach(Base):
    """
    A peach is a round fruit with a fuzzy skin, juicy flesh, a large pit in the center, and a sweet, delicate flavor.
    """
    

class PeachIceTeaBox(Base):
    """
    Peach ice tea box refers to the container or packaging used to store and sell peach-flavored iced tea, usually in a ready-to-drink form. The box may contain individual servings or larger quantities, and can be made of various materials such as cardboard, plastic, or glass.
    """
    

class Pear(Base):
    """
    A pear is a sweet, juicy fruit with a bulbous shape, a thin skin, and a core with seeds in the center.
    """
    

class PhillipsScrewdriver(Base):
    """
    A Phillips screwdriver is a hand-held tool with a cross-shaped tip used for driving screws with a corresponding cross-shaped indentation in their head.
    """
    

class PicklesJar(Base):
    ...
    

class PillBox(Base):
    ...
    

class Pitcher(Base):
    """
    A pitcher is a container, usually made of glass or ceramic, with a handle and spout used for holding and pouring beverages such as water, juice, or tea.
    """
    

class PitcherLid(Base):
    """
    A pitcher lid is a cover used to close the opening of a pitcher and prevent the contents from spilling or becoming contaminated.
    """
    

class PlasticChain(Base):
    """
    Plastic chain is a chain made from plastic links that are interconnected to form a continuous chain. It is often used as a visual barrier or for crowd control in public places, as well as for decorative purposes in events, exhibitions, and retail displays.
    """
    

class PlasticKnife(Base):
    """
    A plastic knife is a cutting tool made from plastic material, typically polypropylene or polystyrene, and is lightweight and disposable. It is commonly used for cutting soft foods such as cake or fruit, or as a safer alternative to metal knives in environments where metal utensils are not allowed.
    """
    

class Plum(Base):
    """
    A plum is a small, round or oval-shaped fruit with a smooth, edible skin, juicy, sweet or tart flesh, and a small pit in the center.
    """
    

class PottedMeatCan(Base):
    """
    A can containing, or at least normally intended to contain, potted meat
    """
    

class PowerDrill(Base):
    """
    A power drill is a motorized tool used for making holes in a variety of materials, or for driving screws or bolts into them, often used in woodworking, metalworking, or construction projects.
    """
    

class PredefinedCategory(Base):
    """
    A predefined category refers to a predetermined classification or grouping system that is established in advance. It allows for the organization and categorization of various elements, such as objects, concepts, or data, based on predefined criteria or characteristics.
    
    In the context of the RoboCup, objects are usually categorized into predefined categories, that might not align with the structure of the exisiting ontologie. To handle this information, objects can be assigned a predefined category.
    """
    

class Collection(Base):
    """
    Any container for entities that share one or more common properties. E.g. "stone objects", "the nurses", "the Louvre Aegyptian collection", all the elections for the Italian President of the Republic. 
    A collection is not a logical class: a collection is a first-order entity, while a class is second-order.
    A collection is neither an aggregate of its member entities (see e.g. ObjectAggregate class).
    """
    

class Pringles(Base):
    ...
    

class PringlesChipsCan(Base):
    """
    Pringles chips can refers to a cylindrical container that holds Pringles brand potato chips.
    """
    

class Pullover(Base):
    ...
    

class Racquetball(Base):
    """
    A racquetball ball is a small, hollow rubber ball used in the sport of racquetball, designed to be bouncy and to move quickly in the enclosed court.
    """
    

class RaspberryJuice(Base):
    """
    Raspberry juice is a beverage made by pressing or extracting the natural liquid from raspberries, a sweet and tart fruit. It is often consumed as a refreshing drink or used as an ingredient in cocktails, smoothies, and other beverages. Raspberry juice is also believed to be a good source of vitamin C and other nutrients.
    """
    

class RaspberryJuiceBox(Base):
    """
    A raspberry juice box is a small, rectangular-shaped container typically made of cardboard or plastic, designed to hold and dispense raspberry juice. It often includes a straw or a screw-on cap for easy access and is a popular choice for on-the-go consumption.
    """
    

class RedWine(Base):
    ...
    

class RoboCupTwoZeroTwoThreeCategories(Base):
    """
    Categories of objects in RoboCup@Home 2023 Bordeaux
    """
    

class RoboCupChallenge(Base):
    """
    The RoboCup@Home competition consists of a series of challenges/tests which the robots have to solve, and the specific tests will change over the years to become more advanced. The performance of the robots is evaluated based on a score derived from competition rules and evaluation by a jury.
    
    The challenges in the RoboCup@Home competition are designed to test a variety of skills relevant to domestic service robots.
    """
    

class Plan(Base):
    """
    A Description having an explicit Goal, to be achieved by executing the plan
    """
    

class Rope(Base):
    """
    A rope is a length of thick, strong, twisted or braided fibers or cords, typically made of natural or synthetic materials, used for tying, lifting, pulling, or securing objects or people.
    """
    

class RubiksCube(Base):
    """
    A Rubik's Cube is a three-dimensional puzzle game that consists of a cube with six faces, each made up of nine smaller squares of different colors, and the object of the game is to twist and turn the cube to solve the puzzle by aligning the colors on each face.
    """
    

class Salt(Base):
    """
    Salt is a mineral substance that is composed of sodium and chloride ions and is commonly used as a seasoning and preservative in food. It is an essential nutrient for the human body and is found in many natural sources such as seawater and rock formations.
    """
    

class Sandwich(Base):
    ...
    

class Sausages(Base):
    ...
    

class Scissors(Base):
    """
    Scissors are a cutting tool consisting of two blades joined together by a pivot, used for cutting paper, fabric, hair, or other materials.
    """
    

class ScotchBriteDobieSponge(Base):
    """
    A Scotch Brite Dobie sponge is a small, rectangular sponge with a scrubbing surface on one side, designed for cleaning dishes and surfaces.
    """
    

class Screw(Base):
    """
    A screw is a threaded metal or wood fastener with a pointed tip used for attaching two objects together or for securing an object in place.
    """
    

class ScrubCleaner(Base):
    ...
    

class ServingMat(Base):
    """
    A serving mat is a type of table mat used to protect surfaces from spills and stains while serving food or drinks. They are typically made from materials such as fabric, plastic, or silicone and come in various shapes, sizes, and designs to suit different occasions and styles.
    """
    

class Shampoo(Base):
    ...
    

class ShelfLayer(Base):
    """
    A shelf board is a flat piece of wood, metal, or glass which is attached to a wall or to the sides of a cupboard. Shelves are used for keeping things on.
    """
    

class Shirt(Base):
    ...
    

class Shoes(Base):
    ...
    

class Sink(Base):
    """
    A sink is a bowl-shaped fixture used for washing hands, dishes, and other small objects, typically located in a kitchen or bathroom.
    """
    

class Skillet(Base):
    """
    A skillet is a flat-bottomed pan with slanted sides and a long handle, used for frying, sautéing, or searing foods.
    """
    

class SkilletLid(Base):
    """
    A skillet lid is a cover made of metal or glass that fits onto a skillet and is used to prevent splatters and retain heat while cooking.
    """
    

class SmallMarker(Base):
    """
    A small marker is a pen-like writing tool with a narrow, pointed tip used for writing or drawing with precision, often used for labeling, highlighting, or coloring.
    """
    

class Soap(Base):
    ...
    

class SoccerBall(Base):
    ...
    

class Socks(Base):
    ...
    

class SoftBall(Base):
    """
    A soft ball is a ball made of a softer, lighter material than a regulation baseball, often used in recreational or amateur leagues, or in games played by children.
    """
    

class Soup(Base):
    """
    Soup is a liquid dish made by cooking vegetables, meat, or fish in water or stock, and often seasoned with herbs and spices. It is a versatile dish that can be served hot or cold, as an appetizer or main course, and is enjoyed around the world for its comforting and nourishing qualities.
    """
    

class Spam(Base):
    ...
    

class Sprite(Base):
    ...
    

class StackingBlocks(Base):
    """
    Stacking blocks are a popular children's toy consisting of various sized and shaped blocks that can be stacked on top of each other to form towers or structures. They are often made of wood, plastic or foam, and are used to improve fine motor skills, hand-eye coordination, and problem-solving abilities in young children.
    """
    

class StapelChips(Base):
    ...
    

class Steak(Base):
    ...
    

class Strawberry(Base):
    """
    A strawberry is a small, red fruit with a juicy texture, sweet taste, and tiny seeds on the surface, commonly used in desserts or as a snack.
    """
    

class StrawberryJello(Base):
    ...
    

class Stroopwafels(Base):
    ...
    

class StudyRoom(Base):
    """
    A room dedicated to studying and working, mostly by reading or writing.
    """
    

class Sugar(Base):
    ...
    

class Sweater(Base):
    ...
    

class TableCloth(Base):
    """
    A tablecloth is a large, rectangular piece of fabric or vinyl used to cover and protect a dining table from spills and stains, as well as to add aesthetic appeal to the setting.
    """
    

class TeaBagBox(Base):
    """
    A tea bag box is a container used for storing and organizing tea bags, typically made of cardboard or other durable materials and featuring separate compartments for different types or flavors of tea. These boxes are commonly used in households, offices, and restaurants to keep tea bags readily available for brewing.
    """
    

class TennisBall(Base):
    """
    A tennis ball is a small, round ball made of rubber with a fuzzy, felt-like outer layer, used in the sport of tennis, as well as in other games, such as fetch for dogs.
    """
    

class TicTac(Base):
    ...
    

class Tie(Base):
    ...
    

class TigerCup(Base):
    ...
    

class Timer(Base):
    """
    A timer is a device used to measure and indicate the passing of a set amount of time.
    """
    

class Toast(Base):
    ...
    

class Tomato(Base):
    """
    A tomato is a fruit that is commonly used as a culinary vegetable and is known for its juicy flesh, bright color, and versatile flavor.
    """
    

class Vegetable(Base):
    """
    A vegetable is a plant or part of a plant that is typically eaten as food and is not classified as a fruit, seed, or spice.
    """
    

class TomatoSoup(Base):
    """
    Tomato soup is a type of soup made from tomatoes, typically with additional ingredients such as broth, cream, and spices. It is a popular comfort food that is enjoyed around the world and can be served hot or cold.
    """
    

class TomatoSoupCan(Base):
    """
    Tomato soup can refers to a cylindrical metal container that holds condensed or ready-to-eat tomato soup.
    """
    

class Toothpaste(Base):
    """
    Toothpaste is a gel or paste used for cleaning teeth and maintaining oral hygiene. It typically contains abrasives, fluoride, and other active ingredients that help remove plaque, prevent tooth decay, and freshen breath.
    """
    

class ToothpasteTube(Base):
    """
    A toothpaste tube is a soft plastic or aluminum container that holds toothpaste for brushing teeth. The tube is typically sealed with a screw-on cap or flip-top lid and is squeezed to dispense the toothpaste.
    """
    

class Tube(Base):
    """
    A tube is a cylindrical container made of plastic, metal, or other materials that is used to hold and dispense a product such as toothpaste, lotion, or paint. Tubes are typically sealed at one end and have an opening at the other end for dispensing the product.
    """
    

class Towel(Base):
    ...
    

class ToyAirplane(Base):
    """
    A toy airplane is a miniature version of an airplane designed for children to play with.
    """
    

class ToyotaHSR(Base):
    """
    Toyota's HSR is a compact mobile cobot (collaborative robot) platform that can move around and fetch objects using a multi-DOF arm while avoiding obstacles. Operable by voice command or by tablet PC, HSR has a highly maneuverable, compact, and lightweight cylindrical body with a folding arm that enables it to pick up objects off the floor, suction up thin objects, retrieve objects from high locations, open curtains, and perform other household tasks.
    """
    

class TrashBag(Base):
    """
    A bag made of plastic or other materials that is used to contain waste material.
    """
    

class TrashBin(Base):
    """
    A container for holding waste or garbage.
    """
    

class TrashCan(Base):
    ...
    

class Tray(Base):
    """
    A flat, shallow container with a raised rim, typically used for carrying food and drink, or for holding small items or loose material.
    """
    

class TropicalJuice(Base):
    ...
    

class TropicalJuiceBottle(Base):
    """
    A tropical juice bottle is a container, typically made of plastic or glass, specifically designed to store and dispense juices made from tropical fruits. It usually features a screw-top or snap cap for freshness and ease of use, and is popular for its portability, making it suitable for on-the-go consumption.
    """
    

class Trousers(Base):
    ...
    

class Tshirt(Base):
    """
    A T-shirt is a type of unisex fabric shirt named after the T shape of its body and sleeves. It is typically made of cotton and features short sleeves and a round neckline. It is a casual and comfortable piece of clothing that is often worn for everyday activities.
    """
    

class Tuna(Base):
    """
    Tuna is a saltwater fish that belongs to the mackerel family. It is a popular seafood item and is commonly used in a variety of dishes such as sushi, salads, and sandwiches.
    """
    

class TunaFishCan(Base):
    """
    Tuna fish can refers to a metal container used to store and preserve tuna fish.
    """
    

class TvTable(Base):
    ...
    

class Vest(Base):
    ...
    

class WashCloth(Base):
    ...
    

class Washer(Base):
    """
    A washer is a thin, flat ring-shaped object, typically made of metal, rubber, or plastic, used to distribute the load of a threaded fastener such as a screw or bolt, or as a spacer, seal, or locking device.
    """
    

class Water(Base):
    ...
    

class WaterBottle(Base):
    ...
    

class Weight(Base):
    """
    Weight refers to the amount of force exerted on an object due to gravity, usually measured in units such as pounds or kilograms. It is a physical property that can change based on the location of the object and the gravitational force acting upon it.
    """
    

class HasFavouriteDrink(Base):
    """
     A preferred drink by a customer 
    """
    


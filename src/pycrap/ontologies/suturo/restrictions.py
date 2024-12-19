from .dependencies import *
from .classes import *
from .individuals import *


Location.is_a = [SpatioTemporalRole]

Room.is_a = [PhysicalPlace]

Entity.is_a = [Thing]

Robot.is_a = [Thing]

Appliance.is_a = [DesignedArtifact]

BakedGood.is_a = [Dish]

Blade.is_a = [DesignedComponent]

Bottle.is_a = [DesignedContainer]

Bowl.is_a = [Crockery, has_predefined_name.value('bowl')]

RoboCupDishes.is_a = [RoboCupTwoZeroTwoThreeCategories]

Fragility.is_a = [Disposition]

Box.is_a = [DesignedContainer, has_shape_region.some(BoxShape)]

Bread.is_a = [BakedGood]

BreakfastPlate.is_a = [Plate]

CerealBox.is_a = [Box]


RoboCupSnacks.is_a = [RoboCupTwoZeroTwoThreeCategories, has_grasp_pose.value('front')]

Cup.is_a = [Crockery, has_physical_component.some(DesignedHandle)]

Cupboard.is_a = [DesignedContainer, DesignedFurniture, has_physical_component.some(Rack)]

Cutlery.is_a = [Tableware]

HouseholdHardware.is_a = [DesignedTool]

Kitchen.is_a = [Room, has_disposition.some(Containment)]

CuttingTool.is_a = [DesignedTool, has_disposition.some(Shaping)]

Deposition.is_a = [Disposition, affords_bearer.only(Deposit), affords_trigger.only(DepositedObject)]

DesignedChair.is_a = [DesignedFurniture, has_disposition.exactly(1, CanBeSatOn)]

DesignedComponent.is_a = [FunctionalPart, DesignedArtifact, has_disposition.some(Connectivity)]

DesignedContainer.is_a = [DesignedArtifact, has_disposition.some(Containment)]

DesignedFurniture.is_a = [DesignedArtifact]

DesignedHandle.is_a = [DesignedComponent, has_disposition.some(Graspability)]

DesignedTool.is_a = [DesignedArtifact]

DinnerPlate.is_a = [Plate]

Dish.is_a = [DesignedArtifact]

Dishwasher.is_a = [Appliance, DesignedContainer]

DishwasherTab.is_a = [DesignedSubstance]

RoboCupCleaningSupplies.is_a = [RoboCupTwoZeroTwoThreeCategories]

Disposition.is_a = [Extrinsic, is_described_by.exactly(1, Affordance), is_described_by.exactly(1, And([Affordance, defines_bearer.exactly(1, Role), defines_event_type.exactly(1, EventType), defines_trigger.exactly(1, Role)]))]

Door.is_a = [DesignedComponent]

Drawer.is_a = [DesignedComponent, DesignedContainer]

Fluid.is_a = [Substance]

Fork.is_a = [Cutlery, has_physical_component.some(DesignedHandle)]

Glass.is_a = [Crockery]

Intrinsic.is_a = [PhysicalQuality]

Jar.is_a = [DesignedContainer]

KitchenCabinet.is_a = [Cupboard]

Cabinet.is_a = [DesignedFurniture]

KitchenKnife.is_a = [Knife]

Knife.is_a = [CuttingTool, has_physical_component.some(Blade), has_physical_component.some(DesignedHandle)]

Lid.is_a = [DesignedComponent]

MilkBottle.is_a = [Bottle]

Milk.is_a = [Drink, has_origin_location.only(Refrigerator), has_predefined_name.value('milk'), has_grasp_pose.value("front")
             , has_location.some(Refrigerator)]

RoboCupDrinks.is_a = [Drink, RoboCupTwoZeroTwoThreeCategories, has_grasp_pose.value('front')]

MilkPack.is_a = [Pack]

Pack.is_a = [DesignedContainer]

Pan.is_a = [Crockery]

Pancake.is_a = [BakedGood]

PastaBowl.is_a = [Bowl]

Plate.is_a = [Crockery]

Pot.is_a = [Crockery]

Preference.is_a = [SocialQuality, is_preference_of.only(Agent)]

Rack.is_a = [DesignedComponent]

Refrigerator.is_a = [Appliance, DesignedContainer, has_physical_component.some(Door)]

Fridge.is_a = [Appliance, DesignedContainer, has_physical_component.some(Door)]


SaladBowl.is_a = [Bowl]

SaltShaker.is_a = [Shaker]

Shaker.is_a = [DesignedContainer]

Sink.is_a = [DesignedComponent]

Sofa.is_a = [DesignedFurniture, has_disposition.exactly(1, CanBeSatOn)]

Spatula.is_a = [Cutlery, has_physical_component.some(DesignedHandle), has_physical_component.some(DesignedSpade)]

Spoon.is_a = [Cutlery, has_disposition.some(Containment), has_physical_component.some(Bowl), has_physical_component.some(DesignedHandle), has_disposition.exactly(1, Insertion), has_disposition.exactly(1, And([Insertion, affords_trigger.only(classifies.only(Substance))]))]

Table.is_a = [DesignedFurniture]

TrashContainer.is_a = [DesignedContainer]

WaterGlass.is_a = [Glass]

WineBottle.is_a = [Bottle]

WineGlass.is_a = [Glass]

AbrasiveSponge.is_a = [Sponge, RoboCupCleaningSupplies, has_predefined_name.value('abrasive sponge')]

Sponge.is_a = [CleaningTool, RoboCupCleaningSupplies, has_predefined_name.value('sponge')]

AdjustableWrench.is_a = [Wrench]

Wrench.is_a = [ForceConcentratingTool, has_physical_component.some(DesignedHandle)]

Apple.is_a = [Fruit, RoboCupFruits, has_predefined_name.value('apple')]

Fruit.is_a = [Food, is_in_room.only(Kitchen), has_predefined_name.value('fruit')]

RoboCupFruits.is_a = [RoboCupTwoZeroTwoThreeCategories]

AppleJuice.is_a = [Juice, has_predefined_name.value('apple juice')]

Juice.is_a = [Drink, has_predefined_name.value('juice')]

Arena.is_a = [Room, has_predefined_name.value('arena')]

Bag.is_a = [DesignedContainer, has_predefined_name.value('bag')]

Balcony.is_a = [Room, has_predefined_name.value('balcony')]

Ball.is_a = [DesignedTool]

Banana.is_a = [Fruit, RoboCupFruits, has_predefined_name.value('banana')]

Baseball.is_a = [Ball, RoboCupToys, has_predefined_name.value('baseball')]

RoboCupToys.is_a = [RoboCupTwoZeroTwoThreeCategories]

Bathroom.is_a = [Room, has_predefined_name.value('bathroom')]

Bed.is_a = [DesignedFurniture, has_predefined_name.value('bed')]

Bedroom.is_a = [Room, has_predefined_name.value('bedroom')]

Beer.is_a = [Drink, has_predefined_name.value('beer')]

Drink.is_a = [Liquid, has_predefined_name.value('drink')]

Bib.is_a = [DesignedFabric, has_predefined_name.value('bib')]

DesignedFabric.is_a = [DesignedArtifact]

BlankCreditCard.is_a = [CreditCard, has_predefined_name.value('blank credit card')]

CreditCard.is_a = [DesignedTool, has_predefined_name.value('credit card')]

BleachCleanserBottle.is_a = [CleanserBottle, RoboCupCleaningSupplies, has_predefined_name.value('bleach'), has_predefined_name.value('bleach cleanser bottle')]

CleanserBottle.is_a = [Bottle, has_predefined_name.value('cleanser bottle')]

Block.is_a = [DesignedComponent]

Blouse.is_a = [Clothing, has_predefined_name.value('blouse')]

Clothing.is_a = [DesignedFabric]

BlueJeans.is_a = [Clothing, has_predefined_name.value('blue jeans')]

Bolt.is_a = [Fastener, MetalHardware, has_predefined_name.value('bolt')]

Fastener.is_a = [HouseholdHardware, has_predefined_name.value('fastener')]

MetalHardware.is_a = [DesignedTool]

Book.is_a = [PrintedMatter]

PrintedMatter.is_a = [DesignedArtifact]

Bookcase.is_a = [Shelf, has_predefined_name.value('book case')]

Shelf.is_a = [Rack, has_predefined_name.value('shelf'), has_predefined_name.value('shelf unit')]

BoxLid.is_a = [Lid]

Brand.is_a = [SocialObject]

SocialObject.is_a = [Object, is_expressed_by.some(InformationObject), has_part.only(SocialObject)]

Can.is_a = [DesignedContainer, has_predefined_name.value('can')]

Candle.is_a = [RoboCupDecorations, DesignedSubstance, has_predefined_name.value('candle')]

RoboCupDecorations.is_a = [RoboCupTwoZeroTwoThreeCategories]

DesignedSubstance.is_a = [DesignedArtifact, FunctionalSubstance]

Candy.is_a = [RoboCupSnacks, Sweets, has_predefined_name.value('candy')]

Sweets.is_a = [ProcessedFood, has_predefined_name.value('sweets')]

Cereal.is_a = [ProcessedFood, has_predefined_name.value('cereal')]
#Cereal.has_predefined_name = ["cereal"]

ProcessedFood.is_a = [Dish, Food, has_predefined_name.value('processed food')]

CerealBowl.is_a = [Bowl, has_predefined_name.value('cereal bowl')]

CerealBoxRoboCup.is_a = [CerealBox, RoboCupSnacks, has_predefined_name.value('cereal box robocup')]

Chain.is_a = [Fastener]

Chair.is_a = [DesignedChair, has_predefined_name.value('chair')]

CheezIt.is_a = [ProcessedFood, has_predefined_name.value('cheezit')]

CheezItCrackerBox.is_a = [CrackerBox, has_predefined_name.value('cheezit cracker box')]

CrackerBox.is_a = [Box, RoboCupSnacks, has_predefined_name.value('cracker box')]

Chili.is_a = [ProcessedFood, has_predefined_name.value('chili')]

ChiliConCarne.is_a = [Chili, has_predefined_name.value('chili con carne')]

ChiliSinCarne.is_a = [Chili, has_predefined_name.value('chili sin carne')]

ChipsCan.is_a = [Can, has_predefined_name.value('chips can')]

ChocolateJello.is_a = [Jello, has_predefined_name.value('chocolate jello')]

Jello.is_a = [DesignedSubstance]

Clamp.is_a = [ForceConcentratingTool, has_physical_component.some(DesignedHandle), has_predefined_name.value('clamp')]

ForceConcentratingTool.is_a = [DesignedTool]

CleaningProduct.is_a = [DesignedSubstance]

CleaningTool.is_a = [DesignedTool]

Cleanser.is_a = [CleaningProduct, has_predefined_name.value('cleanser')]

ClearBox.is_a = [Box, has_predefined_name.value('clear box')]

CloseAction.is_a = [Action]

ClosedState.is_a = [State]

Coat.is_a = [Clothing, has_predefined_name.value('coat')]

CoatHanger.is_a = [Rack, has_predefined_name.value('coat hanger')]

Rack.is_a = [DesignedFurniture, has_predefined_name.value('rack')]

CoatRack.is_a = [Rack, Rack, has_predefined_name.value('coat rack')]

CocoaBox.is_a = [Box, has_predefined_name.value('cocoa box')]

Coffee.is_a = [Drink, has_predefined_name.value('coffee')]

CoffeeCan.is_a = [Can, RoboCupFood, has_predefined_name.value('coffee can')]

RoboCupFood.is_a = [RoboCupTwoZeroTwoThreeCategories]

CoffeeGrounds.is_a = [DesignedSubstance, has_predefined_name.value('coffee grounds')]

CoffeePack.is_a = [Pack, has_predefined_name.value('coffee pack')]

CoffeeTable.is_a = [Table, has_predefined_name.value('coffee table')]

Cola.is_a = [RoboCupDrinks, Drink, has_predefined_name.value('cola')]

ColaBottle.is_a = [Cola, RoboCupDrinks, has_predefined_name.value('big coke'), has_predefined_name.value('cola bottle')]

ColaCan.is_a = [Can, Cola, RoboCupDrinks, has_predefined_name.value('coke'), has_predefined_name.value('cola can')]

ColoredWoodBlock.is_a = [WoodBlock, has_predefined_name.value('colored wood block')]

WoodBlock.is_a = [Block, has_predefined_name.value('wood block')]

Condiment.is_a = [ProcessedFood, has_predefined_name.value('condiment')]

Cookie.is_a = [Sweets, has_predefined_name.value('cookie')]

CookieBox.is_a = [Box, has_predefined_name.value('cookie box')]

CookieMix.is_a = [Sweets, has_predefined_name.value('cookie mix')]

Cornflakes.is_a = [Cereal, RoboCupFood, has_predefined_name.value('cornflakes')]

CornyBox.is_a = [Box, RoboCupSnacks, has_predefined_name.value('corny box')]

Couch.is_a = [DesignedFurniture, has_predefined_name.value('couch')]

CouchTable.is_a = [Table, has_predefined_name.value('couch table')]

CrispsBag.is_a = [Bag, RoboCupSnacks, has_predefined_name.value('crips')]

CupBlue.is_a = [Cup]

CupGreen.is_a = [Cup]

CupSmall.is_a = [Cup]

Curry.is_a = [ProcessedFood, RoboCupFood, has_predefined_name.value('curry')]

Customer.is_a = [NaturalPerson]

NaturalPerson.is_a = [Person, PhysicalAgent]

CustomerID.is_a = [Customer]

CuttingBoard.is_a = [CuttingTool, has_predefined_name.value('cutting board')]

DesignedArtifact.is_a = [PhysicalArtifact, is_described_by.some(Design)]

Desk.is_a = [Table, has_predefined_name.value('desk')]

DetectAction.is_a = [Action]

Dice.is_a = [Toy, has_predefined_name.value('dice')]

Toy.is_a = [DesignedTool]

DiningRoom.is_a = [Room, has_predefined_name.value('dining room')]

DiningTable.is_a = [Table, has_predefined_name.value('dining table')]

DinnerTable.is_a = [Table, has_predefined_name.value('dinner table')]

DishwasherTray.is_a = [Drawer, has_predefined_name.value('dishwasher tray')]

DominoSugarBox.is_a = [SugarBox, has_predefined_name.value('domino sugar box')]

SugarBox.is_a = [Box, RoboCupFood, has_predefined_name.value('sugar box')]

Dress.is_a = [Clothing, has_predefined_name.value('dress')]

Liquid.is_a = [Fluid, Food, has_disposition.only(Perishable), has_predefined_name.value('liquid')]

DubbelFris.is_a = [AppleJuice, RoboCupDrinks, has_predefined_name.value('dubbelfris')]

Fanta.is_a = [RoboCupDrinks, Drink]

FantaCan.is_a = [Fanta, RoboCupDrinks, has_predefined_name.value('fanta')]

Fish.is_a = [Food, has_predefined_name.value('fish')]

Food.is_a = [BiologicalObject]

FlatScrewdriver.is_a = [ScrewDriver]

ScrewDriver.is_a = [ForceConcentratingTool, has_physical_component.some(DesignedHandle), has_predefined_name.value('screwdriver')]

Flavor.is_a = [Intrinsic]

FoamBrick.is_a = [Block, has_predefined_name.value('foam brick')]

BiologicalObject.is_a = [PhysicalBody]

Footlocker.is_a = [DesignedContainer]

FrenchsYellowMustardBottle.is_a = [MustardBottle, has_predefined_name.value('frenchs yellow mustard bottle')]

MustardBottle.is_a = [Bottle, RoboCupFood, has_predefined_name.value('mustard bottle')]

GelatineBox.is_a = [Box, has_predefined_name.value('gelatine box')]

GinTonic.is_a = [Drink, has_predefined_name.value('gin tonic')]

GingerAle.is_a = [Drink, has_predefined_name.value('ginger ale')]

GlassCleanerSprayBottle.is_a = [SprayBottle, has_predefined_name.value('glass cleaner spray bottle')]

SprayBottle.is_a = [Bottle, has_predefined_name.value('spray bottle')]

Gloves.is_a = [Clothing, has_predefined_name.value('gloves')]

GolfBall.is_a = [Ball, has_predefined_name.value('golf ball')]

Grapes.is_a = [Fruit, RoboCupFruits, has_predefined_name.value('grapes')]

GroceryBag.is_a = [Bag, has_predefined_name.value('grocery bag')]

Hagelslag.is_a = [ProcessedFood, RoboCupFood, has_predefined_name.value('chocolate sprinkles'), has_predefined_name.value('hagelslag')]

Hallway.is_a = [Room, has_predefined_name.value('hallway')]

HallwayCabinet.is_a = [Cabinet, has_predefined_name.value('hallway cabinet')]

Hammer.is_a = [ForceConcentratingTool, has_physical_component.some(DesignedHandle), has_predefined_name.value('hammer')]

HansanoWeidemilchMilkPack.is_a = [MilkPack]

Hat.is_a = [Clothing, has_predefined_name.value('hat')]

Hoodie.is_a = [Clothing, has_predefined_name.value('hoodie')]

HotChocolate.is_a = [Drink, has_predefined_name.value('hot chocolate')]

HygieneProduct.is_a = [DesignedSubstance]

IceTea.is_a = [RoboCupDrinks, Tea, Drink, has_predefined_name.value('ice tea'), has_predefined_name.value('iced tea')]

Tea.is_a = [Drink, has_predefined_name.value('tea')]

IceTeaBottle.is_a = [Bottle, IceTea, RoboCupDrinks, has_predefined_name.value('ice tea bottle')]

IceTeaBox.is_a = [Box]

IceTeaCan.is_a = [IceTea, RoboCupDrinks, has_predefined_name.value('ice tea')]

IcedCoffee.is_a = [Coffee, has_predefined_name.value('iced coffee')]

JaFettarmeHMilchMilkPack.is_a = [MilkPack]

Jacket.is_a = [Clothing, has_predefined_name.value('jacket')]

JellOChocolatePuddingBox.is_a = [PuddingBox, RoboCupFood, has_predefined_name.value('jello chocolate pudding box')]

PuddingBox.is_a = [Box, has_predefined_name.value('pudding box')]

JellOStrawberryBox.is_a = [JelloBox, RoboCupFood, has_predefined_name.value('jello strawberry box')]

JelloBox.is_a = [Box, has_predefined_name.value('jello box')]

Jug.is_a = [DesignedContainer, has_predefined_name.value('jug')]

JuiceBottle.is_a = [Bottle, has_predefined_name.value('juice bottle')]

JuiceBox.is_a = [Box, has_predefined_name.value('juice box')]

JuicePack.is_a = [Pack, RoboCupDrinks, has_predefined_name.value('dubbelfris'), has_predefined_name.value('juice pack')]

KelloggsCornFlakes.is_a = [Cereal, has_predefined_name.value('kelloggs corn flakes')]

KelloggsCornFlakesBox.is_a = [CerealBox]

Ketchup.is_a = [Condiment, has_predefined_name.value('ketchup')]

KitchenCounter.is_a = [Table, has_predefined_name.value('kitchen counter')]

KitchenTable.is_a = [Table, has_predefined_name.value('kitchen table')]

Kiwi.is_a = [Fruit, RoboCupFruits, has_predefined_name.value('kiwi')]

KoellnKnusperSchokoKeksBox.is_a = [CerealBox]

Lamp.is_a = [DesignedFurniture, has_predefined_name.value('lamp')]

LargeMarker.is_a = [WritingTool, has_predefined_name.value('large marker')]

WritingTool.is_a = [DesignedTool, has_predefined_name.value('writing tool')]

Leggings.is_a = [Clothing, has_predefined_name.value('leggings')]

Lego.is_a = [Toy, has_predefined_name.value('lego')]

LegoDuplo.is_a = [Lego, has_predefined_name.value('lego duplo')]

Lemon.is_a = [Fruit, RoboCupFruits, has_predefined_name.value('lemon')]

Lemonade.is_a = [Drink, has_predefined_name.value('lemonade')]

Perishable.is_a = [Disposition]

Liquorice.is_a = [RoboCupSnacks, Sweets, has_predefined_name.value('liquorice')]

LivingRoom.is_a = [Room, has_predefined_name.value('living room')]

Lobby.is_a = [Room, has_predefined_name.value('lobby')]

LookAtAction.is_a = [Action]

LoungeChair.is_a = [DesignedChair, has_predefined_name.value('lounge chair')]

LowFatLongLastingMilk.is_a = [Milk, has_predefined_name.value('low fat long lasting milk')]

Magazine.is_a = [PrintedMatter]

Marble.is_a = [Ball, has_predefined_name.value('marble')]

MasterchefCan.is_a = [Can]

Mayonaise.is_a = [Condiment, RoboCupFood, has_predefined_name.value('mayonaise')]

Meat.is_a = [Food, has_predefined_name.value('meat')]

MetalBowl.is_a = [Bowl, has_predefined_name.value('metal bowl')]

MetalCup.is_a = [Cup, has_predefined_name.value('metal cup')]

MetalKnife.is_a = [Knife, has_predefined_name.value('metal knife')]

MetalMug.is_a = [Mug, has_predefined_name.value('metal mug')]

Mug.is_a = [Cup, has_predefined_name.value('mug')]

MetalPlate.is_a = [Plate]

Microwave.is_a = [Appliance, DesignedContainer, has_predefined_name.value('microwave')]

MilkCoffee.is_a = [Coffee, has_predefined_name.value('milk coffee')]

MilkGlass.is_a = [Glass, has_predefined_name.value('milk glass')]

MilkJug.is_a = [Jug, has_predefined_name.value('milk jug')]

MilkPackJa.is_a = [MilkPack]

Mineral.is_a = [Substance]

Substance.is_a = [PhysicalBody]

MineralWater.is_a = [Drink, has_predefined_name.value('mineral water')]

MiniSoccerBall.is_a = [Ball, RoboCupToys, has_predefined_name.value('mini soccer ball')]

MoveTorsoAction.is_a = [Action]

Muesli.is_a = [ProcessedFood, has_predefined_name.value('muesli')]

MuesliBox.is_a = [Box, has_predefined_name.value('mueslibox')]

MulledWineTea.is_a = [Tea, has_predefined_name.value('mulled wine tea')]

Mustard.is_a = [Condiment, has_predefined_name.value('mustard')]

Nail.is_a = [Fastener, MetalHardware, has_predefined_name.value('nail')]

Napkin.is_a = [DesignedFabric, has_predefined_name.value('napkin')]

NavigateAction.is_a = [Action]

NesquickCerealBox.is_a = [CerealBox, has_predefined_name.value('nesquick cereal box')]

NinePegHoleTest.is_a = [Toy, has_predefined_name.value('nine peg hole test')]

Nut.is_a = [Fastener, MetalHardware, has_predefined_name.value('nut')]

Office.is_a = [Room, has_predefined_name.value('office')]

OpenAction.is_a = [Action]

OpenedState.is_a = [State]

Orange.is_a = [Fruit, RoboCupFruits, has_predefined_name.value('orange')]

OrangeJuice.is_a = [Juice, has_predefined_name.value('orange juice')]

OrangeJuiceBox.is_a = [JuiceBox, RoboCupDrinks, has_predefined_name.value('orange juice box')]

Oregano.is_a = [Spice, has_predefined_name.value('oregano')]

Spice.is_a = [Food, has_predefined_name.value('spice')]

OreganoShaker.is_a = [Shaker, has_predefined_name.value('oregano shaker')]

Package.is_a = [DesignedContainer]

PadlockAndKeys.is_a = [HouseholdHardware, has_predefined_name.value('padlock and keys')]

PancakeMix.is_a = [ProcessedFood, RoboCupFood, has_predefined_name.value('pancake mix')]

PancakeMixBottle.is_a = [Bottle, has_predefined_name.value('pancake mix bottle')]

Pantry.is_a = [Room, has_predefined_name.value('pantry')]

Pants.is_a = [Clothing, has_predefined_name.value('pants')]

PaperBag.is_a = [Bag, has_predefined_name.value('paper bag')]

ParkArmsAction.is_a = [Action]

PastryBox.is_a = [Box, has_predefined_name.value('pastry box')]

PeaSoupCan.is_a = [RoboCupFood, SoupCan, has_predefined_name.value('pea soup')]

SoupCan.is_a = [Can, has_predefined_name.value('soup can')]

Peach.is_a = [Fruit, RoboCupFruits, has_predefined_name.value('peach')]

PeachIceTeaBox.is_a = [IceTeaBox, has_predefined_name.value('peach ice tea box')]

Pear.is_a = [Fruit, RoboCupFruits, has_predefined_name.value('pear')]

PhillipsScrewdriver.is_a = [ScrewDriver]

PicklesJar.is_a = [Jar, has_predefined_name.value('pickles jar')]

PillBox.is_a = [Box, has_predefined_name.value('pill box')]

Pitcher.is_a = [DesignedContainer, has_predefined_name.value('pitcher')]

PitcherLid.is_a = [Lid]

PlasticChain.is_a = [Chain, has_predefined_name.value('plastic chain')]

PlasticKnife.is_a = [Knife, has_predefined_name.value('plastic knife')]

Plum.is_a = [Fruit, RoboCupFruits, has_predefined_name.value('plum')]

PottedMeatCan.is_a = [Can, RoboCupFood, has_predefined_name.value('potted meat can')]

PowerDrill.is_a = [ForceConcentratingTool]

PredefinedCategory.is_a = [Collection]

Collection.is_a = [SocialObject, has_part.only(Collection)]

Pringles.is_a = [ProcessedFood]

PringlesChipsCan.is_a = [ChipsCan, RoboCupSnacks, has_predefined_name.value('pringles')]

Pullover.is_a = [Clothing, has_predefined_name.value('pullover')]

Racquetball.is_a = [Ball, has_predefined_name.value('racquet ball')]

RaspberryJuice.is_a = [Juice, has_predefined_name.value('raspberry juice')]

RaspberryJuiceBox.is_a = [JuiceBox, has_predefined_name.value('raspberry juice box')]

RedWine.is_a = [Drink, has_predefined_name.value('red wine')]

RoboCupTwoZeroTwoThreeCategories.is_a = [PredefinedCategory]

RoboCupChallenge.is_a = [Plan]

Plan.is_a = [Description, has_component.some(Goal)]

Rope.is_a = [Fastener, has_predefined_name.value('rope')]

RubiksCube.is_a = [RoboCupToys, Toy, has_predefined_name.value('rubiks cube')]

Salt.is_a = [Mineral, Spice, has_predefined_name.value('salt')]

Sandwich.is_a = [ProcessedFood, has_predefined_name.value('sandwich')]

Sausages.is_a = [Meat, RoboCupFood, has_predefined_name.value('sausages')]

Scissors.is_a = [CuttingTool, has_physical_component.some(Blade), has_predefined_name.value('scissors')]

ScotchBriteDobieSponge.is_a = [AbrasiveSponge, has_predefined_name.value('scotch brite dobie sponge')]

Screw.is_a = [Fastener, MetalHardware, has_predefined_name.value('screw')]

ScrubCleaner.is_a = [CleaningProduct, has_predefined_name.value('scrub cleaner')]

ServingMat.is_a = [DesignedFabric, has_predefined_name.value('serving mat')]

Shampoo.is_a = [HygieneProduct, has_predefined_name.value('shampoo')]

ShelfLayer.is_a = [DesignedComponent]

Shirt.is_a = [Clothing, has_predefined_name.value('shirt')]

Shoes.is_a = [Clothing, has_predefined_name.value('shoes')]

Sink.is_a = [Appliance, DesignedContainer, has_predefined_name.value('sink')]

Skillet.is_a = [Pan, has_predefined_name.value('skillet')]

SkilletLid.is_a = [Lid]

SmallMarker.is_a = [WritingTool, has_predefined_name.value('small marker')]

Soap.is_a = [HygieneProduct, RoboCupCleaningSupplies, has_predefined_name.value('soap')]

SoccerBall.is_a = [Ball, has_predefined_name.value('soccer ball')]

Socks.is_a = [Clothing, has_predefined_name.value('socks')]

SoftBall.is_a = [Ball, has_predefined_name.value('soft ball')]

Soup.is_a = [Liquid, ProcessedFood, has_predefined_name.value('soup')]

Spam.is_a = [Meat, has_predefined_name.value('spam')]

Sprite.is_a = [Drink, has_predefined_name.value('sprite')]

StackingBlocks.is_a = [Toy, has_predefined_name.value('stacking blocks')]

StapelChips.is_a = [ChipsCan, has_predefined_name.value('stapel chips')]

Steak.is_a = [Meat, has_predefined_name.value('steak')]

Strawberry.is_a = [Fruit, RoboCupFruits, has_predefined_name.value('strawberry')]

StrawberryJello.is_a = [Jello, has_predefined_name.value('strawberry jello')]

Stroopwafels.is_a = [RoboCupSnacks, Sweets, has_predefined_name.value('stroopwafels')]

StudyRoom.is_a = [Room, has_predefined_name.value('study room')]

Sugar.is_a = [Spice, has_predefined_name.value('sugar')]

Sweater.is_a = [Clothing, has_predefined_name.value('sweater')]

TableCloth.is_a = [DesignedFabric, has_predefined_name.value('table cloth')]

TeaBagBox.is_a = [Box, has_predefined_name.value('tea bag box')]

TennisBall.is_a = [Ball, RoboCupToys, has_predefined_name.value('tennis ball')]

TicTac.is_a = [Sweets, has_predefined_name.value('tictac')]

Tie.is_a = [Clothing, has_predefined_name.value('tie')]

TigerCup.is_a = [Cup, has_predefined_name.value('tiger cup')]

Timer.is_a = [HouseholdHardware, has_predefined_name.value('timer')]

Toast.is_a = [Bread, has_predefined_name.value('toast')]

Tomato.is_a = [Vegetable, has_predefined_name.value('tomato')]

Vegetable.is_a = [Food, has_predefined_name.value('vegetable')]

TomatoSoup.is_a = [Soup, has_predefined_name.value('tomato soup')]

TomatoSoupCan.is_a = [RoboCupFood, SoupCan, has_predefined_name.value('tomato soup can')]

Toothpaste.is_a = [HygieneProduct, has_predefined_name.value('toothpaste')]

ToothpasteTube.is_a = [Tube]

Tube.is_a = [DesignedContainer]

Towel.is_a = [DesignedFabric, has_predefined_name.value('towel')]

ToyAirplane.is_a = [Toy, has_predefined_name.value('toy airplane')]

ToyotaHSR.is_a = [Robot, has_predefined_name.value('toyota HSR')]

TrashBag.is_a = [TrashContainer]

TrashBin.is_a = [TrashContainer, has_predefined_name.value('trash bin')]

TrashCan.is_a = [TrashContainer, has_predefined_name.value('trashcan')]

Tray.is_a = [HouseholdHardware, has_predefined_name.value('tray')]

TropicalJuice.is_a = [Juice, has_predefined_name.value('tropical juice')]

TropicalJuiceBottle.is_a = [JuiceBottle, RoboCupDrinks, has_predefined_name.value('tropical juice bottle')]

Trousers.is_a = [Clothing, has_predefined_name.value('trousers')]

Tshirt.is_a = [Clothing, has_predefined_name.value('tshirt')]

Tuna.is_a = [Fish, has_predefined_name.value('tuna')]

TunaFishCan.is_a = [Can, RoboCupFood, has_predefined_name.value('tuna fish can')]

TvTable.is_a = [Table, has_predefined_name.value('tv table')]

Vest.is_a = [Clothing, has_predefined_name.value('vest')]

WashCloth.is_a = [CleaningTool, RoboCupCleaningSupplies, has_predefined_name.value('washcloth')]

Washer.is_a = [MetalHardware, has_predefined_name.value('washer')]

Water.is_a = [RoboCupDrinks, Drink]

WaterBottle.is_a = [Water, RoboCupDrinks, Drink, has_predefined_name.value('water'), has_predefined_name.value('water bottle')]

Weight.is_a = [Intrinsic]

HasFavouriteDrink.is_a = [Preference]

has_name_string.is_a = [DatatypeProperty, has_data_value]
has_name_string.domain = [Entity]
has_name_string.range = [str]

has_position_data.is_a = [DatatypeProperty, has_space_parameter]
has_position_data.domain = [SpaceRegion, SpaceRegion]
has_position_data.range = [str, str]

handled.is_a = [DatatypeProperty, has_data_value]
handled.domain = [Entity]
handled.range = [bool]

has_data_value.is_a = [DatatypeProperty]
has_data_value.domain = [Entity]

has_confidence_value.is_a = [DatatypeProperty, has_data_value]
has_confidence_value.domain = [Entity]
has_confidence_value.range = [float]

has_data_source.is_a = [DatatypeProperty, has_data_value]
has_data_source.domain = [Entity]
has_data_source.range = [str]

has_grasp_pose.is_a = [DatatypeProperty, has_data_value]

has_handle_state.is_a = [DatatypeProperty, has_data_value]
has_handle_state.domain = [Entity]
has_handle_state.range = [bool]

has_position.is_a = [DatatypeProperty, has_data_value]

has_predefined_name.is_a = [DatatypeProperty, has_name_string]

has_robocup_name.is_a = [DatatypeProperty, has_name_string]

has_disposition.is_a = [ObjectProperty, has_quality]
has_disposition.domain = [Object, Object]
has_disposition.range = [Disposition, Disposition]

has_physical_component.is_a = [ObjectProperty, has_component]
has_physical_component.domain = [PhysicalObject]
has_physical_component.range = [PhysicalObject]

has_destination_location.is_a = [ObjectProperty, has_predefined_location]

has_predefined_location.is_a = [ObjectProperty, associated_with]

has_origin_location.is_a = [ObjectProperty, has_predefined_location]
has_origin_location.domain = [Entity]
has_origin_location.range = [Entity]

associated_with.is_a = [ObjectProperty, SymmetricProperty, TransitiveProperty]
associated_with.domain = [Entity]
associated_with.range = [Entity]

is_entry_to.is_a = [ObjectProperty, has_location]
is_entry_to.domain = [Location]
is_entry_to.range = [Room]

has_location.is_a = [ObjectProperty, associated_with]
has_location.domain = [Entity]
has_location.range = [Entity]

is_exit_from.is_a = [ObjectProperty, has_location]
is_exit_from.domain = [Location]
is_exit_from.range = [Room]

is_in_room.is_a = [ObjectProperty, has_predefined_location]


rule = Imp(namespace=ontology)
rule.set_as_rule(
       """
       Milk(?m), Refrigerator(?b) -> has_origin_location(?m, ?b)
       """
   )

rule.set_as_rule(
    """
    Milk(?m), Fridge(?b) -> has_origin_location(?m, ?b)
    """
)
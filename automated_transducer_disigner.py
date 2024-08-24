from abaqus import *
from abaqusConstants import *
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *

backwardCompatibility.setValues(includeDeprecated=True, reportDeprecated=False)


# this method creat a section by vertical and horizontal lines in a part model
# input parameters:
# @param part_model : an instance of @ModelAxiSymmetricPart
# @param x_section : an instance of @list . the x coordinates of vertical sections
# @param y_section : an instance of @list . the y coordinates of horizontal sections
# @param max_x : an instance of @int or @float . the maximum radius or an upper line of radius of part
# @param max_y : an instance of @int or @float . the maximum height or an upper line of height of part
# results:
# add @partition_sketch_key and @partition_sketch in part model
def creat_section(part_model, x_section, y_section, max_x, max_y):
    part_model.partition_sketch_key = 'sketch_partitions' + "_" + part_model.part_key
    part_model.partition_sketch = part_model.model.ConstrainedSketch(
        gridSpacing=1, name=part_model.partition_sketch_key, sheetSize=part_model.sheet_size,
        transform=part_model.part.MakeSketchTransform(sketchPlane=part_model.part.faces[0], sketchPlaneSide=SIDE1,
                                                      sketchOrientation=RIGHT, origin=(0.0, 0.0, 0.0)))
    part_model.part.projectReferencesOntoSketch(filter=COPLANAR_EDGES, sketch=part_model.partition_sketch)
    for y in x_section:
        part_model.partition_sketch.Line(point1=(0, y), point2=(max_x, y))
    for x in y_section:
        part_model.partition_sketch.Line(point1=(x, 0), point2=(x, max_y))
    part_model.part.PartitionFaceBySketch(faces=part_model.part.faces.getSequenceFromMask(('[#1 ]',), ),
                                          sketch=part_model.partition_sketch)


class ModelMaterialForModalAnalysis:
    __count = 0

    ALUMINIUM_6061_T6 = {"material_name": "AL6061T6", "density": 2.7e-9, "elastic_module": 68900, "poisson_ratio": 0.33}
    PZT4 = {"material_name": "PZT4", "density": 2.517e-9, "elastic_module": 67400, "poisson_ratio": 0.3}
    ST37 = {"material_name": "ST37", "density": 7.7e-9, "elastic_module": 200000, "poisson_ratio": 0.29}
    SCREW_12_9 = {"material_name": "SCREW_12_9", "density": 7.85e-9, "elastic_module": 206000, "poisson_ratio": 0.29}
    COPPER = {"material_name": "COPPER", "density": 8.93e-9, "elastic_module": 110000, "poisson_ratio": 0.343}

    @staticmethod
    def standard_material(model, material):
        return ModelMaterialForModalAnalysis(model=model, material_name=material["material_name"],
                                             density=material["density"], elastic_module=material["elastic_module"],
                                             poisson_ratio=material["poisson_ratio"])

    def __init__(self, model, material_name, density, elastic_module, poisson_ratio):
        self.__material = model.Material(name=material_name)
        self.material.Density(table=((density,),))
        self.material.Elastic(table=((elastic_module, poisson_ratio),))
        self.__section_name = "section" + "_" + material_name
        self.__section = model.HomogeneousSolidSection(material=material_name, name=self.section_name, thickness=None)
        self.__model = model
        self.__material_name = material_name
        self.__density = density
        self.__elastic_modules = elastic_module
        self.__poisson_ratio = poisson_ratio

    @property
    def model(self):
        return self.__model

    @property
    def material_name(self):
        return self.__material_name

    @property
    def density(self):
        return self.__density

    @property
    def elastic_module(self):
        return self.__elastic_modules

    @property
    def poisson_ratio(self):
        return self.__poisson_ratio

    @property
    def material(self):
        return self.__material

    @property
    def section(self):
        return self.__section

    @property
    def section_name(self):
        return self.__section_name


class ModelAxiSymmetricPart:
    __count = 0

    def __init__(self, model, part_key, sheet_size=200):
        ModelAxiSymmetricPart.__count += 1
        self.__model = model
        self.__part_key = part_key
        self.__part = model.Part(name=self.part_key, dimensionality=AXISYMMETRIC, type=DEFORMABLE_BODY)
        self.__sketch = model.ConstrainedSketch(name='sketch_' + self.part_key, sheetSize=sheet_size)
        self.sketch.sketchOptions.setValues(viewStyle=AXISYM)
        self.sketch.ConstructionLine(point1=(0.0, -100.0), point2=(0.0, 100.0))
        self._partition_sketch_key = None
        self._partition_sketch = None
        self._material = None
        self.__sheet_size = sheet_size
        self._all_faces = None

    @property
    def model(self):
        return self.__model

    @property
    def part(self):
        return self.__part

    @property
    def sketch(self):
        return self.__sketch

    @property
    def part_key(self):
        return self.__part_key

    @property
    def partition_sketch_key(self):
        return self._partition_sketch_key

    @property
    def partition_sketch(self):
        return self._partition_sketch

    @property
    def sheet_size(self):
        return self.__sheet_size

    @property
    def material(self):
        return self._material

    @property
    def all_faces(self):
        return self._all_faces


class ModelDisk(ModelAxiSymmetricPart):
    __count = 0

    def __init__(self, model, inner_diameter, outer_diameter, thickness, part_key):
        ModelDisk.__count += 1
        ModelAxiSymmetricPart.__init__(self, model=model, part_key=part_key,
                                       sheet_size=4 * max(inner_diameter, outer_diameter, thickness))
        self.__inner_diameter = inner_diameter
        self.__outer_diameter = outer_diameter
        self.__thickness = thickness
        self.sketch.rectangle(point1=(inner_diameter / 2., 0), point2=(outer_diameter / 2., thickness))
        self.part.BaseShell(sketch=self.sketch)
        self._all_faces = self.part.Set(faces=self.part.faces, name="all_faces" + "_" + part_key)
        self.__surface_key_top = "surface_top_" + self.part_key
        self.__surface_top = self.part.Surface(
            name=self.surface_key_top,
            side1Edges=self.part.edges.findAt((((inner_diameter + outer_diameter) / 4., thickness, 0.),), ))
        self.__surface_key_bottom = "surface_bottom_" + self.part_key
        self.__surface_bottom = self.part.Surface(
            name=self.surface_key_bottom,
            side1Edges=self.part.edges.findAt((((inner_diameter + outer_diameter) / 4., 0., 0.),), ))

    @property
    def inner_diameter(self):
        return self.__inner_diameter

    @property
    def outer_diameter(self):
        return self.__outer_diameter

    @property
    def thickness(self):
        return self.__thickness

    @property
    def surface_key_top(self):
        return self.__surface_key_top

    @property
    def surface_top(self):
        return self.__surface_top

    @property
    def surface_key_bottom(self):
        return self.__surface_key_bottom

    @property
    def surface_bottom(self):
        return self.__surface_bottom


class ModelPiezoelectric(ModelDisk):
    __count = 0

    def __init__(self, model, inner_diameter=15, outer_diameter=45, thickness=5, material=None):
        ModelPiezoelectric.__count += 1
        part_key = "Piezoelectric" + "_" + str(ModelPiezoelectric.__count)
        ModelDisk.__init__(self, model=model, inner_diameter=inner_diameter, outer_diameter=outer_diameter,
                           thickness=thickness, part_key=part_key)
        if material is None:
            material = ModelMaterialForModalAnalysis.standard_material(model=model,
                                                                       material=ModelMaterialForModalAnalysis.PZT4)
        self._material = material
        self.part.SectionAssignment(offset=0.0, offsetField='', offsetType=MIDDLE_SURFACE, region=self.all_faces,
                                    sectionName=material.section_name, thicknessAssignment=FROM_SECTION)


class ModelElectrode(ModelDisk):
    __count = 0

    def __init__(self, model, inner_diameter=15, outer_diameter=45, thickness=0.3, material=None):
        ModelElectrode.__count += 1
        part_key = "Electrode" + "_" + str(ModelElectrode.__count)
        ModelDisk.__init__(self, model=model, inner_diameter=inner_diameter, outer_diameter=outer_diameter,
                           thickness=thickness, part_key=part_key)
        if material is None:
            material = ModelMaterialForModalAnalysis.standard_material(model=model,
                                                                       material=ModelMaterialForModalAnalysis.COPPER)
        self._material = material
        self.part.SectionAssignment(offset=0.0, offsetField='', offsetType=MIDDLE_SURFACE, region=self.all_faces,
                                    sectionName=material.section_name, thicknessAssignment=FROM_SECTION)


class ModelBacking(ModelAxiSymmetricPart):
    __count = 0

    def __init__(self, model, length, screw_box_length, outer_diameter=45, screw_diameter=12, screwdriver_diameter=18,
                 piezoelectric_inner_diameter=15, material=None):
        ModelBacking.__count += 1
        part_key = "Backing" + "_" + str(ModelBacking.__count)
        ModelAxiSymmetricPart.__init__(self, model=model, part_key=part_key,
                                       sheet_size=4 * max(length, outer_diameter))
        if material is None:
            material = ModelMaterialForModalAnalysis.standard_material(model=model,
                                                                       material=ModelMaterialForModalAnalysis.ST37)
        self._material = material
        self.sketch.Line(point1=((screw_diameter / 2.) + 0.1, 0), point2=(outer_diameter / 2., 0))
        self.sketch.Line(point1=(outer_diameter / 2., 0), point2=(outer_diameter / 2., length))
        self.sketch.Line(point1=(outer_diameter / 2., length), point2=((screwdriver_diameter / 2.) + 0.1, length))
        self.sketch.Line(point1=((screwdriver_diameter / 2.) + 0.1, length),
                         point2=((screwdriver_diameter / 2.) + 0.1, length - screw_box_length))
        self.sketch.Line(point1=((screwdriver_diameter / 2.) + 0.1, length - screw_box_length),
                         point2=((screw_diameter / 2.) + 0.1, length - screw_box_length))
        self.sketch.Line(point1=((screw_diameter / 2.) + 0.1, length - screw_box_length),
                         point2=((screw_diameter / 2.) + 0.1, 0))
        self.part.BaseShell(sketch=self.sketch)
        self._all_faces = self.part.Set(faces=self.part.faces, name="all_faces" + "_" + part_key)
        self.part.SectionAssignment(offset=0.0, offsetField='', offsetType=MIDDLE_SURFACE, region=self.all_faces,
                                    sectionName=material.section_name, thicknessAssignment=FROM_SECTION)
        self.part.PartitionEdgeByParam(
            edges=self.part.edges.findAt(((screw_diameter + 0.2) / 2., length - screw_box_length, 0), ),
            parameter=1 - (float(0.2) / float(screwdriver_diameter - screw_diameter)))
        self.part.PartitionEdgeByParam(
            edges=self.part.edges.findAt((float(outer_diameter - screw_diameter - 0.1) / 2., 0., 0.), ),
            parameter=1 - (float(piezoelectric_inner_diameter - screw_diameter - 0.2) / float(
                outer_diameter - screw_diameter - 0.2)))
        self.__surface_key_backing_to_piezoelectric_contact = "surface_backing_piezoelectric_" + self.part_key
        self.__surface_backing_to_piezoelectric_contact = self.part.Surface(
            name=self.surface_key_backing_to_piezoelectric_contact,
            side1Edges=self.part.edges.findAt((((outer_diameter + piezoelectric_inner_diameter) / 4., 0., 0.),), ))
        self.__surface_key_backing_to_screw_contact = "surface_backing_screw_" + self.part_key
        self.__surface_backing_to_screw_contact = self.part.Surface(
            name=self.surface_key_backing_to_screw_contact,
            side1Edges=self.part.edges.findAt(
                (((screw_diameter + screwdriver_diameter + 0.4) / 4., length - screw_box_length, 0.),), ))
        self.__length = length
        self.__outer_diameter = outer_diameter
        self.__screw_diameter = screw_diameter
        self.__screw_box_length = screw_box_length
        self.__screwdriver_diameter = screwdriver_diameter
        creat_section(part_model=self, x_section=[length - screw_box_length], y_section=[], max_x=outer_diameter,
                      max_y=length)

    @property
    def length(self):
        return self.__length

    @property
    def outer_diameter(self):
        return self.__outer_diameter

    @property
    def screw_diameter(self):
        return self.__screw_diameter

    @property
    def screw_box_length(self):
        return self.__screw_box_length

    @property
    def screwdriver_diameter(self):
        return self.__screwdriver_diameter

    @property
    def surface_key_backing_to_piezoelectric_contact(self):
        return self.__surface_key_backing_to_piezoelectric_contact

    @property
    def surface_backing_to_piezoelectric_contact(self):
        return self.__surface_backing_to_piezoelectric_contact

    @property
    def surface_key_backing_to_screw_contact(self):
        return self.__surface_key_backing_to_screw_contact

    @property
    def surface_backing_to_screw_contact(self):
        return self.__surface_backing_to_screw_contact


class ModelMatching(ModelAxiSymmetricPart):
    __count = 0

    def __init__(self, model, length, diameter=45, screw_diameter=12, screw_hole_length=11.5,
                 screw_engagement_length=8.75, piezoelectric_inner_diameter=15, material=None):
        ModelMatching.__count += 1
        part_key = "Matching" + "_" + str(ModelMatching.__count)
        ModelAxiSymmetricPart.__init__(self, model=model, part_key=part_key,
                                       sheet_size=4 * max(length, diameter, screw_diameter, screw_hole_length))
        if material is None:
            material = ModelMaterialForModalAnalysis.standard_material(
                model=model, material=ModelMaterialForModalAnalysis.ALUMINIUM_6061_T6)
        self._material = material
        self.sketch.Line(point1=(0, 0), point2=(diameter / 2., 0))
        self.sketch.Line(point1=(diameter / 2., 0), point2=(diameter / 2., length))
        self.sketch.Line(point1=(diameter / 2., length), point2=(screw_diameter / 2., length))
        self.sketch.Line(point1=(screw_diameter / 2., length),
                         point2=(screw_diameter / 2., length - screw_hole_length))
        self.sketch.Line(point1=(screw_diameter / 2., length - screw_hole_length),
                         point2=(0, length - screw_hole_length))
        self.sketch.Line(point1=(0, length - screw_hole_length), point2=(0, 0))
        self.part.BaseShell(sketch=self.sketch)
        self._all_faces = self.part.Set(faces=self.part.faces, name="all_faces" + "_" + part_key)
        self.part.SectionAssignment(offset=0.0, offsetField='', offsetType=MIDDLE_SURFACE, region=self.all_faces,
                                    sectionName=material.section_name, thicknessAssignment=FROM_SECTION)
        self.part.PartitionEdgeByParam(
            edges=self.part.edges.findAt((screw_diameter / 2., length - (screw_hole_length / 2.), 0.), ),
            parameter=screw_engagement_length / screw_hole_length)
        self.part.PartitionEdgeByParam(
            edges=self.part.edges.findAt(((screw_diameter / 2.) + ((diameter - screw_diameter) / 4.), length, 0.), ),
            parameter=1 - (float(piezoelectric_inner_diameter - screw_diameter) / float(diameter - screw_diameter)))
        self.__surface_key_matching_to_screw_contact = "surface_matching_screw_" + self.part_key
        self.__surface_matching_to_screw_contact = self.part.Surface(
            name=self.surface_key_matching_to_screw_contact,
            side1Edges=self.part.edges.findAt(((screw_diameter / 2., length - (screw_engagement_length / 2.), 0.),), ))
        self.__surface_key_matching_to_piezoelectric_contact = "surface_matching_piezoelectric_" + self.part_key
        self.__surface_matching_to_piezoelectric_contact = self.part.Surface(
            name=self.surface_key_matching_to_piezoelectric_contact,
            side1Edges=self.part.edges.findAt((((screw_diameter + diameter) / 4., length, 0.),), ))
        self.__length = length
        self.__diameter = diameter
        self.__screw_diameter = screw_diameter
        self.__screw_hole_length = screw_hole_length
        creat_section(part_model=self, y_section=[diameter / 2.], x_section=[length - screw_hole_length],
                      max_x=diameter, max_y=length)

    @property
    def length(self):
        return self.__length

    @property
    def diameter(self):
        return self.__diameter

    @property
    def screw_diameter(self):
        return self.__screw_diameter

    @property
    def screw_hole_length(self):
        return self.__screw_hole_length

    @property
    def surface_key_matching_to_piezoelectric_contact(self):
        return self.__surface_key_matching_to_piezoelectric_contact

    @property
    def surface_matching_to_piezoelectric_contact(self):
        return self.__surface_matching_to_piezoelectric_contact

    @property
    def surface_key_matching_to_screw_contact(self):
        return self.__surface_key_matching_to_screw_contact

    @property
    def surface_matching_to_screw_contact(self):
        return self.__surface_matching_to_screw_contact


class ModelScrew(ModelAxiSymmetricPart):
    __count = 0

    class StandardScrews(dict):
        def __init__(self):
            dict.__init__(self, {})

    SCREW_DIAMETERS = [1.6, 2, 2.5, 3, 4, 5, 6, 8, 10, 12, 14, 16, 20, 24, 30, 36, 42, 48, 56, 64]
    SCREW_LENGTH = [2.5, 3, 4, 5, 6, 8, 10, 12, 16, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 80, 90, 100, 110, 120,
                    130, 140, 150, 160, 180, 200, 220, 240, 260, 280, 300]
    SCREW_LENGTH_LIMIT = {"1.6": (2.5, 16), "2": (3, 20), "2.5": (4, 25), "3": (5, 30), "4": (6, 40), "5": (8, 50),
                          "6": (10, 60), "8": (12, 80), "10": (16, 100), "12": (20, 120), "14": (25, 140),
                          "16": (25, 160), "20": (30, 200), "24": (40, 200), "30": (45, 200), "36": (55, 200),
                          "42": (60, 300), "48": (70, 300), "56": (80, 300), "64": (90, 300)}
    SCREW_STEP_LENGTH = {"1.6": 0.35, "2": 0.4, "2.5": 0.45, "3": 0.5, "4": 0.7, "5": 0.8, "6": 1, "8": 1.25, "10": 1.5,
                         "12": 1.75, "14": 2, "16": 2, "20": 2.5, "24": 3, "30": 3.5, "36": 4, "42": 4.5, "48": 5,
                         "56": 5.5, "64": 6}
    SCREWDRIVER_DIAMETER = {"1.6": 3, "2": 3.8, "2.5": 4.5, "3": 5.5, "4": 7, "5": 8.5, "6": 10, "8": 13, "10": 16,
                            "12": 18, "14": 21, "16": 24, "20": 30, "24": 36, "30": 45, "36": 54, "42": 63, "48": 72,
                            "56": 84, "64": 96}
    SCREWDRIVER_LENGTH = {"1.6": 1.6, "2": 2, "2.5": 2.5, "3": 3, "4": 4, "5": 5, "6": 6, "8": 8, "10": 10, "12": 12,
                          "14": 14, "16": 16, "20": 20, "24": 24, "30": 30, "36": 36, "42": 42, "48": 48, "56": 56,
                          "64": 64}

    def __init__(self, model, screw_diameter=12, screw_length=25, material=None):
        ModelScrew.__count += 1
        part_key = "Screw" + "_" + str(ModelScrew.__count)
        ModelAxiSymmetricPart.__init__(self, model=model, part_key=part_key,
                                       sheet_size=4 * max(screw_diameter, screw_length,
                                                          ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)],
                                                          ModelScrew.SCREWDRIVER_LENGTH[str(screw_diameter)]))
        if material is None:
            material = ModelMaterialForModalAnalysis.standard_material(
                model=model, material=ModelMaterialForModalAnalysis.SCREW_12_9)
        self._material = material
        self.sketch.Line(point1=(0, 0), point2=(screw_diameter / 2., 0))
        self.sketch.Line(point1=(screw_diameter / 2., 0),
                         point2=(screw_diameter / 2., screw_length))
        self.sketch.Line(point1=(screw_diameter / 2., screw_length),
                         point2=(ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)] / 2., screw_length))
        self.sketch.Line(point1=(ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)] / 2., screw_length),
                         point2=(ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)] / 2.,
                                 screw_length + ModelScrew.SCREWDRIVER_LENGTH[str(screw_diameter)]))
        self.sketch.Line(point1=(ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)] / 2.,
                                 screw_length + ModelScrew.SCREWDRIVER_LENGTH[str(screw_diameter)]),
                         point2=(0, screw_length + ModelScrew.SCREWDRIVER_LENGTH[str(screw_diameter)]))
        self.sketch.Line(point1=(0, screw_length + ModelScrew.SCREWDRIVER_LENGTH[str(screw_diameter)]), point2=(0, 0))
        self.part.BaseShell(sketch=self.sketch)
        self._all_faces = self.part.Set(faces=self.part.faces, name="all_faces" + "_" + part_key)
        self.part.SectionAssignment(offset=0.0, offsetField='', offsetType=MIDDLE_SURFACE, region=self.all_faces,
                                    sectionName=material.section_name, thicknessAssignment=FROM_SECTION)
        self.part.PartitionEdgeByParam(edges=self.part.edges.findAt(
            (float(ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)] + screw_diameter) / 4., screw_length, 0.), ),
            parameter=0.2 / float(ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)] - screw_diameter))
        self.part.PartitionEdgeByParam(edges=self.part.edges.findAt((screw_diameter / 2., screw_length / 2., 0.), ),
                                       parameter=float(
                                           5 * (ModelScrew.SCREW_STEP_LENGTH[str(screw_diameter)])) / screw_length)
        self.__surface_key_screw_to_backing_contact = "surface_screw_backing_" + self.part_key
        self.__surface_screw_to_backing_contact = self.part.Surface(
            name=self.surface_key_screw_to_backing_contact,
            side1Edges=self.part.edges.findAt(
                (((screw_diameter + ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)] - 0.4) / 4., screw_length,
                  0.),), ))
        self.__surface_key_screw_to_matching_contact = "surface_screw_matching_" + self.part_key
        self.__surface_screw_to_matching_contact = self.part.Surface(
            name=self.surface_key_screw_to_matching_contact,
            side1Edges=self.part.edges.findAt(((screw_diameter / 2., 0., 0.),), ))
        self.__screw_diameter = screw_diameter
        self.__length = screw_length
        self.__screwdriver_diameter = ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)]
        self.__screwdriver_length = ModelScrew.SCREWDRIVER_LENGTH[str(screw_diameter)]
        creat_section(part_model=self, x_section=[screw_length], y_section=[],
                      max_x=ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)],
                      max_y=screw_length + ModelScrew.SCREWDRIVER_LENGTH[str(screw_diameter)])

    @property
    def screw_diameter(self):
        return self.__screw_diameter

    @property
    def length(self):
        return self.__length

    @property
    def screwdriver_diameter(self):
        return self.__screwdriver_diameter

    @property
    def screwdriver_length(self):
        return self.__screwdriver_length

    @property
    def surface_key_screw_to_backing_contact(self):
        return self.__surface_key_screw_to_backing_contact

    @property
    def surface_screw_to_backing_contact(self):
        return self.__surface_screw_to_backing_contact

    @property
    def surface_key_screw_to_matching_contact(self):
        return self.__surface_key_screw_to_matching_contact

    @property
    def surface_screw_to_matching_contact(self):
        return self.__surface_screw_to_matching_contact


class ModelTransducer:
    __count = 0

    def __init__(self, length_of_matching, length_of_backing, number_of_piezoelectrics=2,
                 piezoelectric_outer_diameter=45, piezoelectric_inner_diameter=15, piezoelectric_thickness=5,
                 thickness_of_electrode=0.3, material_of_piezoelectric=None, material_of_electrode=None,
                 material_of_screw=None, material_of_matching=None, material_of_backing=None, mesh_size=1):
        ModelTransducer.__count += 1
        self.__model_key = "Transducer" + "_" + str(ModelTransducer.__count)
        self.__model = mdb.Model(name=self.model_key)
        self.__number_of_piezoelectrics = number_of_piezoelectrics
        self.__piezoelectric = ModelPiezoelectric(model=self.model, inner_diameter=piezoelectric_inner_diameter,
                                                  outer_diameter=piezoelectric_outer_diameter,
                                                  thickness=piezoelectric_thickness, material=material_of_piezoelectric)
        self.__electrode = ModelElectrode(model=self.model, inner_diameter=piezoelectric_inner_diameter,
                                          outer_diameter=piezoelectric_outer_diameter, thickness=thickness_of_electrode,
                                          material=material_of_electrode)
        screw_diameter = None
        for i in ModelScrew.SCREW_DIAMETERS:
            if i <= piezoelectric_inner_diameter - 2:
                screw_diameter = i
            else:
                break
        screw_criterion_length = length_of_backing + number_of_piezoelectrics * (
                piezoelectric_thickness + thickness_of_electrode) + 5 * ModelScrew.SCREW_STEP_LENGTH[
                                     str(screw_diameter)] - ModelScrew.SCREWDRIVER_LENGTH[str(screw_diameter)]
        screw_length = None
        for i in ModelScrew.SCREW_LENGTH:
            lower_limit, upper_limit = ModelScrew.SCREW_LENGTH_LIMIT[str(screw_diameter)]
            if upper_limit >= i >= lower_limit:
                if screw_length is None:
                    screw_length = i
                else:
                    if abs(i - screw_criterion_length) < abs(screw_length - screw_criterion_length):
                        screw_length = i
        self.__screw = ModelScrew(model=self.model, screw_diameter=screw_diameter, screw_length=screw_length,
                                  material=material_of_screw)
        self.__matching = ModelMatching(model=self.model, length=length_of_matching,
                                        diameter=piezoelectric_outer_diameter,
                                        screw_diameter=screw_diameter,
                                        screw_hole_length=6 * ModelScrew.SCREW_STEP_LENGTH[str(screw_diameter)] + 1,
                                        material=material_of_matching)
        self.__backing = ModelBacking(model=self.model, length=length_of_backing,
                                      outer_diameter=piezoelectric_outer_diameter, screw_diameter=screw_diameter,
                                      screw_box_length=ModelScrew.SCREWDRIVER_LENGTH[str(screw_diameter)] + (
                                              screw_criterion_length - screw_length),
                                      screwdriver_diameter=ModelScrew.SCREWDRIVER_DIAMETER[str(screw_diameter)],
                                      piezoelectric_inner_diameter=piezoelectric_inner_diameter,
                                      material=material_of_backing)
        self.model.rootAssembly.DatumCsysByThreePoints(coordSysType=CYLINDRICAL, origin=(0.0, 0.0, 0.0),
                                                       point1=(1.0, 0.0, 0.0), point2=(0.0, 0.0, -1.0))
        self.__matching_instance = self.model.rootAssembly.Instance(dependent=OFF,
                                                                    name="instance_of_" + self.matching.part_key,
                                                                    part=self.matching.part)
        self.__piezoelectric_instances = []
        self.__electrode_instances = []
        for i in range(self.number_of_piezoelectrics):
            self.piezoelectric_instances.append(self.model.rootAssembly.Instance(
                dependent=OFF, name="instance" + str(i) + "_of_" + self.piezoelectric.part_key,
                part=self.piezoelectric.part))
            self.model.rootAssembly.translate(
                instanceList=("instance" + str(i) + "_of_" + self.piezoelectric.part_key,),
                vector=(0.0, self.matching.length + i * (self.piezoelectric.thickness + self.electrode.thickness), 0.0))
            self.electrode_instances.append(self.model.rootAssembly.Instance(
                dependent=OFF, name="instance" + str(i) + "_of_" + self.electrode.part_key, part=self.electrode.part))
            self.model.rootAssembly.translate(
                instanceList=("instance" + str(i) + "_of_" + self.electrode.part_key,),
                vector=(0.0,
                        self.matching.length + self.piezoelectric.thickness + i * (
                                self.piezoelectric.thickness + self.electrode.thickness),
                        0.0))
        self.__backing_instance = self.model.rootAssembly.Instance(dependent=OFF,
                                                                   name="instance_of_" + self.backing.part_key,
                                                                   part=self.backing.part)
        self.model.rootAssembly.translate(
            instanceList=("instance_of_" + self.backing.part_key,),
            vector=(0.0,
                    self.matching.length + self.number_of_piezoelectrics * (
                            self.piezoelectric.thickness + self.electrode.thickness),
                    0.0))
        self.__screw_instance = self.model.rootAssembly.Instance(dependent=OFF,
                                                                 name="instance_of_" + self.screw.part_key,
                                                                 part=self.screw.part)
        self.model.rootAssembly.translate(
            instanceList=("instance_of_" + self.screw.part_key,),
            vector=(0.0, self.matching.length + self.number_of_piezoelectrics * (
                    self.piezoelectric.thickness + self.electrode.thickness) + self.backing.length - (
                            self.screw.length + self.backing.screw_box_length),
                    0.0))
        self.__step_key = "frequency_step" + "_" + self.model_key
        self.__step = self.model.FrequencyStep(maxEigen=30000.0, name=self.step_key, previous='Initial')
        self.model.fieldOutputRequests['F-Output-1'].setValues(variables=('S', 'E', 'U'))
        if self.screw.material.elastic_module < self.backing.material.elastic_module:
            master = self.backing_instance.surfaces[self.backing.surface_key_backing_to_screw_contact]
            slave = self.screw_instance.surfaces[self.screw.surface_key_screw_to_backing_contact]
        else:
            slave = self.backing_instance.surfaces[self.backing.surface_key_backing_to_screw_contact]
            master = self.screw_instance.surfaces[self.screw.surface_key_screw_to_backing_contact]
        self.model.Tie(adjust=ON, master=master, name='screw_backing', positionToleranceMethod=COMPUTED, slave=slave,
                       thickness=ON, tieRotations=ON)
        if self.screw.material.elastic_module < self.matching.material.elastic_module:
            master = self.matching_instance.surfaces[self.matching.surface_key_matching_to_screw_contact]
            slave = self.screw_instance.surfaces[self.screw.surface_key_screw_to_matching_contact]
        else:
            slave = self.matching_instance.surfaces[self.matching.surface_key_matching_to_screw_contact]
            master = self.screw_instance.surfaces[self.screw.surface_key_screw_to_matching_contact]
        self.model.Tie(adjust=ON, master=master, name='screw_matching', positionToleranceMethod=COMPUTED, slave=slave,
                       thickness=ON, tieRotations=ON)
        if self.backing.material.elastic_module < self.electrode.material.elastic_module:
            master = self.electrode_instances[-1].surfaces[self.electrode.surface_key_top]
            slave = self.backing_instance.surfaces[self.backing.surface_key_backing_to_piezoelectric_contact]
        else:
            slave = self.electrode_instances[-1].surfaces[self.electrode.surface_key_top]
            master = self.backing_instance.surfaces[self.backing.surface_key_backing_to_piezoelectric_contact]
        self.model.Tie(adjust=ON, master=master, name='backing_electrode', positionToleranceMethod=COMPUTED,
                       slave=slave, thickness=ON, tieRotations=ON)
        if self.matching.material.elastic_module < self.piezoelectric.material.elastic_module:
            master = self.piezoelectric_instances[0].surfaces[self.piezoelectric.surface_key_bottom]
            slave = self.matching_instance.surfaces[self.matching.surface_key_matching_to_piezoelectric_contact]
        else:
            slave = self.piezoelectric_instances[0].surfaces[self.piezoelectric.surface_key_bottom]
            master = self.matching_instance.surfaces[self.matching.surface_key_matching_to_piezoelectric_contact]
        self.model.Tie(adjust=ON, master=master, name='matching_piezoelectric', positionToleranceMethod=COMPUTED,
                       slave=slave, thickness=ON, tieRotations=ON)
        for i in range(number_of_piezoelectrics - 1):
            if self.electrode.material.elastic_module < self.piezoelectric.material.elastic_module:
                master1 = self.piezoelectric_instances[i].surfaces[self.piezoelectric.surface_key_top]
                slave1 = self.electrode_instances[i].surfaces[self.electrode.surface_key_bottom]
                master2 = self.piezoelectric_instances[i + 1].surfaces[self.piezoelectric.surface_key_bottom]
                slave2 = self.electrode_instances[i].surfaces[self.electrode.surface_key_top]
            else:
                slave1 = self.piezoelectric_instances[i].surfaces[self.piezoelectric.surface_key_top]
                master1 = self.electrode_instances[i].surfaces[self.electrode.surface_key_bottom]
                slave2 = self.piezoelectric_instances[i + 1].surfaces[self.piezoelectric.surface_key_bottom]
                master2 = self.electrode_instances[i].surfaces[self.electrode.surface_key_top]
            self.model.Tie(adjust=ON, master=master1, name='top_piezoelectric_bottom_electrode_' + str(i),
                           positionToleranceMethod=COMPUTED, slave=slave1, thickness=ON, tieRotations=ON)
            self.model.Tie(adjust=ON, master=master2, name='bottom_piezoelectric_top_electrode_' + str(i),
                           positionToleranceMethod=COMPUTED, slave=slave2, thickness=ON, tieRotations=ON)
        if self.electrode.material.elastic_module < self.piezoelectric.material.elastic_module:
            master = self.piezoelectric_instances[-1].surfaces[self.piezoelectric.surface_key_top]
            slave = self.electrode_instances[-1].surfaces[self.electrode.surface_key_bottom]
        else:
            slave = self.piezoelectric_instances[-1].surfaces[self.piezoelectric.surface_key_top]
            master = self.electrode_instances[-1].surfaces[self.electrode.surface_key_bottom]
        self.model.Tie(adjust=ON, master=master, name='last_electrode_piezoelectric', positionToleranceMethod=COMPUTED,
                       slave=slave, thickness=ON, tieRotations=ON)
        self.model.rootAssembly.seedPartInstance(
            deviationFactor=0.1, minSizeFactor=0.1,
            regions=tuple([v for k, v in self.model.rootAssembly.instances.items()]), size=mesh_size)
        self.__mesh_size = mesh_size
        all_root_assembly_regions_temp = None
        for k, v in self.model.rootAssembly.instances.items():
            if all_root_assembly_regions_temp is None:
                all_root_assembly_regions_temp = v.faces
            else:
                all_root_assembly_regions_temp += v.faces
        self.model.rootAssembly.setMeshControls(elemShape=QUAD, regions=all_root_assembly_regions_temp,
                                                technique=STRUCTURED)
        self.model.rootAssembly.setElementType(
            elemTypes=(ElemType(
                elemCode=CAX4R, elemLibrary=STANDARD, secondOrderAccuracy=OFF, hourglassControl=ENHANCED,
                distortionControl=DEFAULT), ElemType(elemCode=CAX3, elemLibrary=STANDARD)),
            regions=(all_root_assembly_regions_temp,))
        self.model.rootAssembly.generateMesh(regions=all_root_assembly_regions_temp)
        self.__job_key = 'Job_' + self.model_key
        self.__job = mdb.Job(atTime=None, contactPrint=OFF, description='', echoPrint=OFF, explicitPrecision=SINGLE,
                             getMemoryFromAnalysis=True, historyPrint=OFF, memory=90, memoryUnits=PERCENTAGE,
                             model=self.model_key, modelPrint=OFF, multiprocessingMode=DEFAULT,
                             name=self.job_key, nodalOutputPrecision=SINGLE, numCpus=1, numGPUs=0, queue=None,
                             resultsFormat=ODB, scratch='', type=ANALYSIS, userSubroutine='', waitHours=0,
                             waitMinutes=0)
        self.job.submit(consistencyChecking=OFF)
        self.job.waitForCompletion()
        print("log")
        self.__path_key = "path_" + self.model_key
        self.__path = session.Path(
            name=self.path_key, type=POINT_LIST,
            expression=((piezoelectric_outer_diameter / 2., 0., 0.),
                        (piezoelectric_outer_diameter / 2.,
                         self.matching.length + self.number_of_piezoelectrics * (
                                 self.piezoelectric.thickness + self.electrode.thickness) + self.backing.length,
                         0.)))
        self.__xy_data_key = "xy_data_" + self.model_key
        session.XYDataFromPath(path=self.path, name=self.xy_data_key, includeIntersections=True, shape=UNDEFORMED,
                               pathStyle=PATH_POINTS, labelType=Y_CORD, step=1, frame=2,
                               variable=('U', NODAL, ((COMPONENT, 'U2'),),))

    @property
    def model(self):
        return self.__model

    @property
    def screw(self):
        return self.__screw

    @property
    def electrode(self):
        return self.__electrode

    @property
    def piezoelectric(self):
        return self.__piezoelectric

    @property
    def matching(self):
        return self.__matching

    @property
    def backing(self):
        return self.__backing

    @property
    def number_of_piezoelectrics(self):
        return self.__number_of_piezoelectrics

    @property
    def model_key(self):
        return self.__model_key

    @property
    def step(self):
        return self.__step

    @property
    def step_key(self):
        return self.__step_key

    @property
    def matching_instance(self):
        return self.__matching_instance

    @property
    def backing_instance(self):
        return self.__backing_instance

    @property
    def screw_instance(self):
        return self.__screw_instance

    @property
    def electrode_instances(self):
        return self.__electrode_instances

    @property
    def piezoelectric_instances(self):
        return self.__piezoelectric_instances

    @property
    def mesh_size(self):
        return self.__mesh_size

    @property
    def job(self):
        return self.__job

    @property
    def job_key(self):
        return self.__job_key

    @property
    def path_key(self):
        return self.__path_key

    @property
    def path(self):
        return self.__path

    @property
    def xy_data_key(self):
        return self.__xy_data_key

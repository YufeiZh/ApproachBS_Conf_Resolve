from config import BOTTOM, TOP, DEFAULT_FAP_HEIGHT
import utils.geocalc as geo
import random
import numpy as np
from PIL import Image, ImageDraw

class Tracon():
    ''' The class of TRACON. '''
    def __init__(self, 
                 identifier="", 
                 ctr_lat=0, 
                 ctr_lon=0,
                 tracon_range=30, 
                 elevation=0,
                 apt_id=[],
                 apt_lat=[],
                 apt_lon=[],
                 apt_ele=[],
                 runway_id=None, 
                 runway_thres_lat=None,
                 runway_thres_lon=None,
                 runway_bearing=None, 
                 runway_length=None,
                 fap_lat=None, 
                 fap_lon=None,
                 fap_alt=None):
        # TRACON Name
        self.identifier = identifier
        # Latitude of the center of the circle TRACON area
        self.ctr_lat = ctr_lat
        # Longitude of the center of the circle TRACON area
        self.ctr_lon = ctr_lon
        # Radius of the circle TRACON area
        self.range = tracon_range
        # Elevation of the center
        self.elevation = elevation
        # Minimum area altitude. It is the minimum altitude can be assigned.
        self.min_alt = elevation + BOTTOM
        # Maximum area altitude. It is the maximum altitude can be assigned.
        self.max_alt = elevation + TOP
        # List of airports that the TRACON provides service for
        self.apt_id = apt_id
        # Latitudes of airports
        self.apt_lat = apt_lat
        # Longitudes of airports
        self.apt_lon = apt_lon
        # Elevations of airports
        self.apt_ele = apt_ele
        # Available runways
        self.runway_id = runway_id
        # Runway threshold latitudes
        self.runway_thres_lat = runway_thres_lat
        # Runway threshold longitudes
        self.runway_thres_lon = runway_thres_lon
        # Runway bearings (i.e. the true headings)
        self.runway_bearing = runway_bearing
        # Runway lengths
        self.runway_length = runway_length
        # FAP(Final Approach Point) latitudes. Note that the FAPs are not realistic FAPs.
        self.fap_lat = fap_lat
        # FAP longitudes
        self.fap_lon = fap_lon
        # FAP altitude
        self.fap_alt = fap_alt
        # Restricted area
        self.restrict = []


    def to_dict(self):
        ''' Convert to dictionary. '''
        dict_tracon = self.__dict__
        
        # self.restrict is a list of instances of Restricted. Convert it to a list of dictionaries
        if len(self.restrict) > 0:
            dict_restricted = []
            for ele in self.restrict:
                dict_restricted.append(ele.__dict__())
            dict_tracon["restrict"] = dict_restricted

        return dict_tracon
    

    @classmethod
    def from_dict(cls, dict_data):
        instance = cls()
        instance.__dict__.update(dict_data)

        
        if len(instance.restrict) > 0:
            restrict_data = []
            for ele in instance.restrict:
                restrict_data.append(Restricted.from_dict(ele))
            instance.restrict = restrict_data
        else:
            instance.restrict = []
        return instance


    def add_restricted_area(self, restricted_area):
        # Do not add if there are already 20 restricted areas
        if len(self.restrict) >= 20:
            return
        self.restrict.append(restricted_area)


    def auto_add_airport_restrict(self):
        ''' Load airports in TRACON area as circle restricted areas
            radius=6NM, bottom=0ft, top=airpot_elev+3000 '''
        for i in range(len(self.apt_id)):
            area_id = self.apt_id[i]
            bottom = self.apt_ele[i]
            top = bottom + 3000
            ctr_lat = self.apt_lat[i]
            ctr_lon = self.apt_lon[i]
            new_restrict = Restricted(area_id=area_id, 
                                      area_type="circle", 
                                      bottom=bottom, 
                                      top=top, 
                                      area_args=[ctr_lat, ctr_lon, 5])
            self.add_restricted_area(new_restrict)


    def add_random_restricts(self, num=None, area_type=None, avoid_apt=False):
        if num is None:
            num = random.randint(1, 20)

        if area_type is None:
            for _ in range(num):
                random_type_circle = random.randint(0, 1)
                if random_type_circle == 1:
                    self.add_random_circle_restricted(avoid_apt=avoid_apt)
                else:
                    self.add_random_polygon_restricted(avoid_apt=avoid_apt)
        elif area_type == "circle":
            for _ in range(num):
                self.add_random_circle_restricted(avoid_apt=avoid_apt)
        elif area_type == "polygon":
            for _ in range(num):
                self.add_random_polygon_restricted(avoid_apt=avoid_apt)


    def add_random_circle_restricted(self, avoid_apt=False):
        # Do not add it if there are already 20 restricted area in TRACON
        if len(self.restrict) >= 20:
            return
        
        # Generate area id
        exist_id = []
        for ele in self.restrict:
            exist_id.append(ele.area_id)
        new_id = "AREA"
        for i in range(1, 21):
            if new_id + str(i) not in exist_id:
                new_id += str(i)
                break

        # Generate area top_alt and bottom_alt
        bottom_alt = random.randint(self.min_alt, self.max_alt - 1500)
        top_alt = random.randint(bottom_alt + 1000, self.max_alt)

        # Generate area center lat and lon
        valid = False
        while not valid:
            valid = True
            dist_to_apts = []
            dist_to_faps = []
            bearing_from_ctr = round(random.uniform(0, 360), 1)
            dist_from_ctr = round(random.uniform(0, self.range), 1)
            lat, lon = geo.destination_by_bearing(self.ctr_lat, self.ctr_lon, bearing_from_ctr, dist_from_ctr)
            lat, lon = round(lat, 7), round(lon, 7)

            # Discard and re-generate if avoid_apt=False and the area center is too close to apts or faps
            skip_fap_check = False
            for i in range(len(self.apt_id)):
                dist = geo.distance_between(lat, lon, self.apt_lat[i], self.apt_lon[i])
                if dist < 6.5 and avoid_apt:
                    valid = False
                    skip_fap_check = True
                    break
                else:
                    dist_to_apts.append(dist)
            if skip_fap_check:
                continue
            for ele in self.apt_id:
                break_outer = False
                for i in range(len(self.runway_id[ele])):
                    dist = geo.distance_between(lat, lon, self.fap_lat[ele][i], self.fap_lon[ele][i])
                    if dist < 3.5 and avoid_apt:
                        valid = False
                        break_outer = True
                        break
                    else:
                        dist_to_faps.append(dist)
                if break_outer:
                    break

        # Generate area radius
        min_radius = 1
        min_dist_to_apts = min(dist_to_apts) if dist_to_apts else float('inf')
        min_dist_to_faps = min(dist_to_faps) if dist_to_faps else float('inf')
        max_radius = min(min_dist_to_apts-5, min_dist_to_faps-2, 10) if avoid_apt else 10
        radius = 1
        while True:
            radius = random.gauss((min_radius+max_radius)/2, 2)
            radius = round(radius * 2) / 2
            if min_radius <= radius <= max_radius:
                break

        # Add area
        new_restrict = Restricted(area_id=new_id, area_type="circle", bottom=bottom_alt, top=top_alt, area_args=[lat, lon, radius])
        self.add_restricted_area(new_restrict)
        

    def add_random_polygon_restricted(self, avoid_apt=False, num_vertices=None):
        # Do not add it if there are already 20 restricted area in TRACON
        if len(self.restrict) >= 20:
            return
        
        # Generate area id
        exist_id = []
        for ele in self.restrict:
            exist_id.append(ele.area_id)
        new_id = "AREA"
        for i in range(1, 21):
            if new_id + str(i) not in exist_id:
                new_id += str(i)
                break

        # Generate area top_alt and bottom_alt
        bottom_alt = random.randint(self.min_alt, self.max_alt - 1500)
        top_alt = random.randint(bottom_alt + 1000, self.max_alt)

        # Generate area center lat and lon
        valid = False
        while not valid:
            valid = True
            dist_to_apts = []
            dist_to_faps = []
            bearing_from_ctr = round(random.uniform(0, 360), 1)
            dist_from_ctr = round(random.uniform(0, self.range), 1)
            lat, lon = geo.destination_by_bearing(self.ctr_lat, self.ctr_lon, bearing_from_ctr, dist_from_ctr)
            lat, lon = round(lat, 7), round(lon, 7)

            # Discard and re-generate if avoid_apt=False and the area center is too close to apts or faps

            skip_fap_check = False
            for i in range(len(self.apt_id)):
                dist = geo.distance_between(lat, lon, self.apt_lat[i], self.apt_lon[i])
                if dist < 6.5 and avoid_apt:
                    valid = False
                    skip_fap_check = True
                    break
                else:
                    dist_to_apts.append(dist)
            if skip_fap_check:
                continue
            for ele in self.apt_id:
                break_outer = False
                for i in range(len(self.runway_id[ele])):
                    dist = geo.distance_between(lat, lon, self.fap_lat[ele][i], self.fap_lon[ele][i])
                    if dist < 3.5 and avoid_apt:
                        valid = False
                        break_outer = True
                        break
                    else:
                        dist_to_faps.append(dist)
                if break_outer:
                    break

        # Generate area radius
        min_radius = 1
        min_dist_to_apts = min(dist_to_apts) if dist_to_apts else float('inf')
        min_dist_to_faps = min(dist_to_faps) if dist_to_faps else float('inf')
        max_radius = min(min_dist_to_apts-5, min_dist_to_faps-2, 10) if avoid_apt else 10

        # Generate number of vertices if not specified:
        if num_vertices is None:
            num_vertices = random.randint(3,8)

        # Generate angle steps:
        angle_steps = random_angle_steps(num_vertices)

        # Generate vertices:
        vertices = []
        angle_cums = 0
        for angle in angle_steps:
            angle_cums += angle
            dist_to_ctr = 20
            while dist_to_ctr < min_radius or dist_to_ctr > max_radius:
                dist_to_ctr = max_radius - np.random.exponential(2.5)
            vert_lat, vert_lon = geo.destination_by_bearing(lat, lon, angle_cums, dist_to_ctr)
            vertices.append(vert_lat)
            vertices.append(vert_lon)

        # Rotate the vertices:
        shift = random.randint(0, num_vertices-1)
        vertices_rotate = []
        for i in range(num_vertices):
            shifted_idx = (i + shift) % num_vertices
            vertices_rotate.append(vertices[2 * shifted_idx])
            vertices_rotate.append(vertices[2 * shifted_idx + 1])


        new_restrict = Restricted(area_id=new_id, area_type="polygon", bottom=bottom_alt, top=top_alt, area_args=vertices_rotate)
        self.add_restricted_area(new_restrict)


    def display_tracon(self):
        img = Image.new('RGB', (800, 800), 'white')
        im_px_access = img.load()
        draw = ImageDraw.Draw(img)

        # Draw TRACON area
        left_up_point = (0, 0)
        right_down_point = (800, 800)
        draw.ellipse([left_up_point, right_down_point], width=3, outline='blue')

        # Draw Apts
        for i in range(len(self.apt_id)):
            size=15
            num_star_points=4
            bearing_from_ctr = geo.bearing_from_to(self.ctr_lat, self.ctr_lon, self.apt_lat[i], self.apt_lon[i])
            dist_from_ctr = geo.distance_between(self.ctr_lat, self.ctr_lon, self.apt_lat[i], self.apt_lon[i])
            cx, cy = polar_to_xOy(bearing_from_ctr, dist_from_ctr, self.range)
            # Draw a star to represent an airport
            angle = np.pi / num_star_points  # Angle between the star points
            points_list = []
            for j in range(2 * num_star_points):
                r = size if j % 2 == 0 else size / 3  # Alternate radius for star spikes
                theta = j * angle
                x = cx + r * np.cos(theta)
                y = cy + r * np.sin(theta)
                points_list.append((x, y))
            
            draw.polygon(points_list, outline='magenta', fill='magenta')
            draw.text((cx-12, cy+15), self.apt_id[i], fill='black')


        # Draw restricted areas
        for i in range(len(self.restrict)):
            if self.restrict[i].area_type=="circle":
                rest_ctr_lat = self.restrict[i].area_args[0]
                rest_ctr_lon = self.restrict[i].area_args[1]
                rest_radius = self.restrict[i].area_args[2]
                bearing_from_ctr = geo.bearing_from_to(self.ctr_lat, self.ctr_lon, rest_ctr_lat, rest_ctr_lon)
                dist_from_ctr = geo.distance_between(self.ctr_lat, self.ctr_lon, rest_ctr_lat, rest_ctr_lon)
                rest_ctr_cx, rest_ctr_cy = polar_to_xOy(bearing_from_ctr, dist_from_ctr, self.range)
                draw_radius = rest_radius * 400 / self.range
                left_up_point = (rest_ctr_cx - draw_radius, rest_ctr_cy - draw_radius)
                right_down_point = (rest_ctr_cx + draw_radius, rest_ctr_cy + draw_radius)
                draw.ellipse([left_up_point, right_down_point], width=2, outline='blue')
                draw.text((rest_ctr_cx+draw_radius*np.cos(np.pi/6)-6, rest_ctr_cy-draw_radius*np.sin(np.pi/6)), 
                           self.restrict[i].area_id, 
                           fill="blue")
                draw.text((rest_ctr_cx+draw_radius*np.cos(np.pi/6)-5, rest_ctr_cy-draw_radius*np.sin(np.pi/6)+10), 
                          f"{self.restrict[i].area_bottom_alt} to {self.restrict[i].area_top_alt}", 
                          fill="blue")

            else:
                draw_vertices = []
                for j in range(len(self.restrict[i].area_args)//2):
                    bearing_from_ctr = geo.bearing_from_to(self.ctr_lat, 
                                                           self.ctr_lon, 
                                                           self.restrict[i].area_args[j*2], 
                                                           self.restrict[i].area_args[j*2+1])
                    dist_from_ctr = geo.distance_between(self.ctr_lat, 
                                                         self.ctr_lon, 
                                                         self.restrict[i].area_args[j*2], 
                                                         self.restrict[i].area_args[j*2+1])
                    draw_vertices.append(polar_to_xOy(bearing_from_ctr, dist_from_ctr, self.range))

                draw.polygon(draw_vertices, width=2, outline='blue')
                labelx, labely = draw_vertices[0]
                draw.text((labelx-5, labely), self.restrict[i].area_id, fill="blue")
                draw.text((labelx-5, labely+10), 
                          f"{self.restrict[i].area_bottom_alt} to {self.restrict[i].area_top_alt}", 
                          fill="blue")

        return img



class Restricted():
    def __init__(self, area_id="", area_type="circle", bottom=0, top=3000, area_args=[]):
        # Identifier of the restricted area
        self.area_id = area_id
        # Type: "circle" or "polygon"
        self.area_type = area_type
        # Bottom altitude (MSE)
        self.area_bottom_alt = bottom
        # Top altitude (MSE)
        self.area_top_alt = top
        # If area_type="circle", area_args is of the form [center_lat, center_lon, radius]
        # if area_type="polygon", area_args is of the form [lat1, lon1, lat2, lon2, ..., latn, lonn]
        if len(area_args) > 16:
            print("Too many vertices. 8 at most.")
            return
        self.area_args = area_args


    def __str__(self) -> str:
        return f"AREA ID: {self.area_id}, TYPE: {self.area_type}, RANGE: {self.area_bottom_alt} to {self.area_top_alt} ft, args: {self.area_args}."


    @classmethod
    def from_dict(cls, dict_data):
        instance = cls()
        instance.__dict__.update(dict_data)
        return instance


    def in_area(self, lat, lon, alt):
        if alt < self.area_bottom_alt or alt > self.area_top_alt:
            return False
        if self.area_type == "circle":
            return geo.distance_between(self.area_args[0], self.area_args[1], lat, lon) <= self.area_args[2]
        return geo.in_polygon(lat, lon, self.area_args)


    def dist_to_area(self, lat, lon):
        if self.in_area(lat, lon, self.area_top):
            return 0
        if self.area_type == "circle":
            return geo.distance_between(self.area_args[0], self.area_args[1], lat, lon) - self.area_args[2]
        return geo.distance_to_polygon(lat, lon, self.area_args)
    
    
    def display(self):

        black = (0, 0, 0)
        white = (255, 255, 255)
        img = Image.new('RGB', (800, 800), white)
        im_px_access = img.load()
        draw = ImageDraw.Draw(img)

        # either use .polygon(), if you want to fill the area with a solid colour
        #draw.polygon(vertices, outline=black, fill=white)

        # or .line() if you want to control the line thickness, or use both methods together!
        if self.area_type=="circle":
            left_up_point = (400-self.area_args[2]*10, 400-self.area_args[2]*10)
            right_down_point = (400+self.area_args[2]*10, 400+self.area_args[2]*10)
            draw.ellipse([left_up_point, right_down_point], outline=black)
        else:
            lat_mean = np.mean([self.area_args[2*i] for i in range(len(self.area_args)//2)])
            lon_mean = np.mean([self.area_args[2*i+1] for i in range(len(self.area_args)//2)])
            vertices_transform = []
            for i in range(len(self.area_args)//2):
                vertices_transform.append((400+(self.area_args[i*2+1]-lon_mean)*2400,
                                           400-(self.area_args[i*2]-lat_mean)*2400))

            draw.polygon(vertices_transform, width=2, outline=black)

        img.show()



def random_angle_steps(steps, irregularity=0.2):
    """Generates the division of a circumference in random angles.

    origin: https://stackoverflow.com/questions/8997099/algorithm-to-generate-random-2d-polygon

    Args:
        steps (int):
            the number of angles to generate.
        irregularity (float):
            variance of the spacing of the angles between consecutive vertices.
    Returns:
        List[float]: the list of the random angles.
    """
    while irregularity is None:
        irregularity = np.random.exponential(0.2)
        if irregularity > 1:
            irregularity = None

    # generate n angle steps
    irregularity *= 360 / steps
    angles = []
    lower = (360 / steps) - irregularity
    upper = (360 / steps) + irregularity
    cumsum = 0
    for i in range(steps):
        angle = random.uniform(lower, upper)
        angles.append(angle)
        cumsum += angle

    # normalize the steps so that point 0 and point n+1 are the same
    cumsum /= (360)
    for i in range(steps):
        angles[i] /= cumsum
    return angles


def polar_to_xOy(theta, rho, tracon_range, scale=400):
    ''' Convert bearing_from_center and dist_from_center to the xOy coordinate
     to display it. The default scale (half size of the image) is 400 (pixels) '''
    theta = -theta+90
    cx = scale + rho * np.cos(np.radians(theta)) * scale / tracon_range
    cy = scale - rho * np.sin(np.radians(theta)) * scale / tracon_range

    return cx, cy
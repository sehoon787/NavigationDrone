import math, random
import cv2
from socket import *
import time

# ["landing point", latitude, longitude, cx, cy]
targetdata = [["0", 35.132833, 129.106215], ["1", 35.133118, 129.105832],
              ["2", 35.133139, 129.106001], ["3", 35.133150, 129.106168],
              ["4", 35.133168, 129.106331], ["5", 35.133181, 129.106487],
              ["6", 35.132965, 129.105877], ["7", 35.132983, 129.106019],
              ["8", 35.132994, 129.106182], ["9", 35.133008, 129.106357],
              ["10", 35.132985, 129.106512], ["11", 35.132681, 129.105871],
              ["12", 35.132695, 129.106040], ["13", 35.132706, 129.106222],
              ["14", 35.132721, 129.106391], ["15", 35.132737, 129.106543],
              ["16", 35.132498, 129.105905], ["17", 35.132498, 129.106048],
              ["18", 35.132516, 129.106236], ["19", 35.132526, 129.106435],
              ["20", 35.132537, 129.106584]]

order = []
# Dynamic하게 바꾸기 가능
client_index = 6    # including home point


class City:
    def __init__(self, x=None, y=None):
        self.x = None
        self.y = None
        if x is not None:
            self.x = x
        else:
            self.x = int(random.random() * 200)
        if y is not None:
            self.y = y
        else:
            self.y = int(random.random() * 200)

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def distanceTo(self, city):
        xDistance = abs(self.getX() - city.getX())
        yDistance = abs(self.getY() - city.getY())
        distance = math.sqrt((xDistance * xDistance) + (yDistance * yDistance))
        return distance

    def __repr__(self):
        return str(self.getX()) + ", " + str(self.getY())


class TourManager:
    destinationCities = []

    def addCity(self, city):
        self.destinationCities.append(city)

    def getCity(self, index):
        return self.destinationCities[index]

    def numberOfCities(self):
        return len(self.destinationCities)


class Tour:
    def __init__(self, tourmanager, tour=None):
        self.tourmanager = tourmanager
        self.tour = []
        self.fitness = 0.0
        self.distance = 0
        if tour is not None:
            self.tour = tour
        else:
            for i in range(0, self.tourmanager.numberOfCities()):
                self.tour.append(None)

    def __len__(self):
        return len(self.tour)

    def __getitem__(self, index):
        return self.tour[index]

    def __setitem__(self, key, value):
        self.tour[key] = value

    def __repr__(self):
        geneString = ''
        for i in range(0, self.tourSize()):
            geneString += str(self.getCity(i)) + ', '
        geneString
        return geneString

    def generateIndividual(self):
        for cityIndex in range(0, self.tourmanager.numberOfCities()):
            self.setCity(cityIndex, self.tourmanager.getCity(cityIndex))
        random.shuffle(self.tour)

    def getCity(self, tourPosition):
        return self.tour[tourPosition]

    def setCity(self, tourPosition, city):
        self.tour[tourPosition] = city
        self.fitness = 0.0
        self.distance = 0

    def getFitness(self):
        if self.fitness == 0:
            self.fitness = 1 / float(self.getDistance())
        return self.fitness

    def getDistance(self):
        if self.distance == 0:
            tourDistance = 0
            for cityIndex in range(0, self.tourSize()):
                fromCity = self.getCity(cityIndex)
                destinationCity = None
                if cityIndex + 1 < self.tourSize():
                    destinationCity = self.getCity(cityIndex + 1)
                else:
                    destinationCity = self.getCity(0)
                tourDistance += fromCity.distanceTo(destinationCity)
            self.distance = tourDistance
        return self.distance

    def tourSize(self):
        return len(self.tour)

    def containsCity(self, city):
        return city in self.tour


class Population:
    def __init__(self, tourmanager, populationSize, initialise):
        self.tours = []
        for i in range(0, populationSize):
            self.tours.append(None)

        if initialise:
            for i in range(0, populationSize):
                newTour = Tour(tourmanager)
                newTour.generateIndividual()
                self.saveTour(i, newTour)

    def __setitem__(self, key, value):
        self.tours[key] = value

    def __getitem__(self, index):
        return self.tours[index]

    def saveTour(self, index, tour):
        self.tours[index] = tour

    def getTour(self, index):
        return self.tours[index]

    def getFittest(self):
        fittest = self.tours[0]
        for i in range(0, self.populationSize()):
            if fittest.getFitness() <= self.getTour(i).getFitness():
                fittest = self.getTour(i)
        return fittest

    def populationSize(self):
        return len(self.tours)


class GA:
    def __init__(self, tourmanager, mutationRate=0.05, tournamentSize=5, elitism=True):
        self.tourmanager = tourmanager
        self.mutationRate = mutationRate
        self.tournamentSize = tournamentSize
        self.elitism = elitism

    def evolvePopulation(self, pop):
        newPopulation = Population(self.tourmanager, pop.populationSize(), False)
        elitismOffset = 0
        if self.elitism:
            newPopulation.saveTour(0, pop.getFittest())
            elitismOffset = 1

        for i in range(elitismOffset, newPopulation.populationSize()):
            parent1 = self.tournamentSelection(pop)
            parent2 = self.tournamentSelection(pop)
            child = self.crossover(parent1, parent2)
            newPopulation.saveTour(i, child)

        for i in range(elitismOffset, newPopulation.populationSize()):
            self.mutate(newPopulation.getTour(i))

        return newPopulation

    def crossover(self, parent1, parent2):
        child = Tour(self.tourmanager)

        startPos = int(random.random() * parent1.tourSize())
        endPos = int(random.random() * parent1.tourSize())
        for i in range(0, child.tourSize()):
            if startPos < endPos and i > startPos and i < endPos:
                child.setCity(i, parent1.getCity(i))
            elif startPos > endPos:
                if not (i < startPos and i > endPos):
                    child.setCity(i, parent1.getCity(i))

        for i in range(0, parent2.tourSize()):
            if not child.containsCity(parent2.getCity(i)):
                for ii in range(0, child.tourSize()):
                    if child.getCity(ii) == None:
                        child.setCity(ii, parent2.getCity(i))
                        break
        return child

    def mutate(self, tour):
        for tourPos1 in range(0, tour.tourSize()):
            if random.random() < self.mutationRate:
                tourPos2 = int(tour.tourSize() * random.random())

                city1 = tour.getCity(tourPos1)
                city2 = tour.getCity(tourPos2)

                tour.setCity(tourPos2, city1)
                tour.setCity(tourPos1, city2)

    def tournamentSelection(self, pop):
        tournament = Population(self.tourmanager, self.tournamentSize, False)
        for i in range(0, self.tournamentSize):
            randomId = int(random.random() * pop.populationSize())
            tournament.saveTour(i, pop.getTour(randomId))
        fittest = tournament.getFittest()
        return fittest

def get_order_From_Client(cnt):
    global order
    orderData = ""
    logdata = "Booking complete!!"

    orderData = connectionSocket.recv(1024)
    orderData = str(orderData).split("b'", 1)[1].rsplit("'", 1)[0]
    order.append(orderData)
    print("Client " + str(cnt+1) + ": " + orderData)

    connectionSocket.send(logdata.encode("utf-8"))


if __name__ == '__main__':

    make_msg = []
    snd_msg = ""
    temp = ""

    print("TSP server with Genetic Algorithm")
    cnt = 0

    ## 1. 소켓통신으로 포인트 5개 받아서 배열에 저장하기
    # TSP server with Genetic Algorithm (Web Koren vm)
    GA_TSP_server_IP = "116.89.189.31"
    GA_TSP_server_PORT = 22041
    GA_TSP = socket(AF_INET, SOCK_STREAM)
    GA_TSP.bind((GA_TSP_server_IP, GA_TSP_server_PORT))
    GA_TSP.listen(1)

    starttime = time.time()
    waiting_time = 0

    while cnt < client_index-1:
    ## get delivery client until 6
    # while cnt < client_index-1:
        print("Waiting Client...")
        connectionSocket, addr = GA_TSP.accept()
        print("Client" + str(cnt+1) + str(addr) + " has connected.")
        get_order_From_Client(cnt)
        cnt = cnt + 1

    # while True:
    #     print("Waiting Client...")
    #     connectionSocket, addr = GA_TSP.accept()
    #     print("Client" + str(client_index+1) + str(addr) + " has connected.")
    #     get_order_From_Client(client_index)
    #     client_index = client_index + 1
    #     waiting_time = time.time() - starttime
    #     print("Remaning time : " + str(waiting_time) + "sec..")
    #     if waiting_time >= 60 or client_index > MAX_CLIENT:
    #         break


    GA_TSP.close()

    # # 임시로
    # order = ["20", "1", "6", "9", "15"]
    order = list(map(str, sorted(list(map(int, order)))))

    n_cities = client_index
    population_size = 50
    n_generations = 100

    # Load the map
    map_original = cv2.imread('./map.jpg')

    # Setup cities and tour
    tourmanager = TourManager()

    ord_cnt = 1
    # draw point
    cx = 278
    cy = 370
    targetdata[0].append(cx)
    targetdata[0].append(cy)
    tourmanager.addCity(City(x=cx, y=cy))
    cv2.circle(map_original, center=(cx, cy), radius=10, color=(0, 0, 0), thickness=-1, lineType=cv2.LINE_AA)
    for i in range(2):
        for j in range(5):
            cx = 85 * (j + 1)
            cy = 133 * (i + 1)
            targetdata[ord_cnt].append(cx)
            targetdata[ord_cnt].append(cy)
            cv2.circle(map_original, center=(cx, cy), radius=10, color=(0, 0, 255), thickness=-1, lineType=cv2.LINE_AA)
            ord_cnt = ord_cnt + 1
    for i in range(2):
        for j in range(5):
            cx = 100 + 85 * j
            cy = 450 + 133 * i
            targetdata[ord_cnt].append(cx)
            targetdata[ord_cnt].append(cy)
            cv2.circle(map_original, center=(cx, cy), radius=10, color=(0, 0, 255), thickness=-1, lineType=cv2.LINE_AA)
            ord_cnt = ord_cnt + 1

    for i in range(len(order)):
        for j in range(len(targetdata)):
            if order[i] == targetdata[j][0]:
                cv2.circle(map_original, center=(targetdata[j][3], targetdata[j][4]),
                           radius=10, color=(0, 0, 0), thickness=-1, lineType=cv2.LINE_AA)
                tourmanager.addCity(City(x=targetdata[j][3], y=targetdata[j][4]))
                break

    # cv2.imshow('map', map_original)
    # cv2.imwrite("./GA.jpg", map_original)
    cv2.imwrite("/root/work/web/GA.jpg", map_original)
    # cv2.waitKey(0)

    # Initialize population
    pop = Population(tourmanager, populationSize=population_size, initialise=True)
    print("Initial cost: " + str(pop.getFittest().getDistance()))

    # Evolve population
    ga = GA(tourmanager)

    for i in range(n_generations):
        pop = ga.evolvePopulation(pop)

        fittest = pop.getFittest()

        map_result = map_original.copy()

        for j in range(1, n_cities):
            cv2.line(
                map_result,
                pt1=(fittest[j - 1].x, fittest[j - 1].y),
                pt2=(fittest[j].x, fittest[j].y),
                color=(255, 0, 0),
                thickness=3,
                lineType=cv2.LINE_AA
            )

        cv2.putText(map_result, org=(10, 25), text='Generation: %d' % (i + 1), fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale=0.7, color=0, thickness=1, lineType=cv2.LINE_AA)
        cv2.putText(map_result, org=(10, 50), text='cost: %.2f' % fittest.getDistance(),
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.7, color=0, thickness=1, lineType=cv2.LINE_AA)
        # cv2.imshow('map', map_result)
        cv2.imwrite("/root/work/web/GA.jpg", map_result)
        # time.sleep(0.1)
        # if cv2.waitKey(100) == ord('q'):
        #     break

    # Print final results
    print("Finished")
    print("Final cost: " + str(pop.getFittest().getDistance()))

    make_msg.append(pop.getFittest())
    make_msg = (str(make_msg[0])).split(', ')
    make_msg = ' '.join(make_msg).split()
    make_msg = list(map(int, make_msg))

    cv2.line(
        map_result,
        pt1=(make_msg[0], make_msg[1]),
        pt2=(make_msg[len(make_msg)-2], make_msg[len(make_msg)-1]),
        color=(255, 0, 0),
        thickness=3,
        lineType=cv2.LINE_AA
    )

    for i in range(len(make_msg)):
        if make_msg[0] == 278 and make_msg[1] == 370:
            break
        else:
            make_msg.append(make_msg[0])
            make_msg.pop(0)

    # cv2.imshow('map', map_result)
    # cv2.imwrite("./GA.jpg", map_result)
    cv2.imwrite("/root/work/web/GA.jpg", map_result)
    # cv2.waitKey(0)

    make_msg.append(make_msg[0])
    make_msg.append(make_msg[1])

    for i in range(len(make_msg)-1):
        for j in range(len(targetdata)):
            if make_msg[i] == targetdata[j][3] and make_msg[i+1] == targetdata[j][4]:
                snd_msg = snd_msg + str(targetdata[j][1]) + "/"
                snd_msg = snd_msg + str(targetdata[j][2]) + "/"
                if i < len(make_msg)-2:
                    temp = temp + targetdata[j][0] + "==>"
                else:
                    temp = temp + targetdata[j][0]
                break

    print("Path : ")
    print(temp)

    snd_msg = str(snd_msg)
    print(snd_msg)

    # Connect Drone client
    GA_TSP_server2_IP = "116.89.189.31"
    GA_TSP_server2_PORT = 22042
    GA_TSP2 = socket(AF_INET, SOCK_STREAM)
    GA_TSP2.bind((GA_TSP_server2_IP, GA_TSP_server2_PORT))
    GA_TSP2.listen(1)
    print("Waiting Drone Client")
    connectionSocket2, addr = GA_TSP2.accept()
    print(str(addr), "has connected.")

    # make message for Drone client
    connectionSocket2.send(snd_msg.encode("utf-8"))
    time.sleep(3)
    connectionSocket2.close()

    print("TSP server finish.")

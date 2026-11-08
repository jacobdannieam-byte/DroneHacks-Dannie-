from .queue import Queue

def add_repair(queue, ticket):
    return queue.add(ticket, "repair")

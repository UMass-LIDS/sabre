import sabre

class CustomReplacement(sabre.Replacement):
    def check_replace(self, quality):
        buffer = self.session.get_buffer_contents()
        for i in range(2, len(buffer)):
            if buffer[i] < quality:
                # return -ve index from end of buffer
                return i - len(buffer)
        # if we arrive here, no switching occurs
        return None

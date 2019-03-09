import sabre

class CustomAbr(sabre.Abr):
    def get_quality_delay(self, segment_index):
        manifest = self.session.manifest
        bitrates = manifest.bitrates
        throughput = self.session.get_throughput()
        quality = 0
        while (quality + 1 < len(bitrates) and
               bitrates[quality + 1] <= throughput):
            quality += 1
        return (quality, 0)
